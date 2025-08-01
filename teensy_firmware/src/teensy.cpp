/*
 * RE-RASSOR Teensy 4.x firmware  – four-wheel diff-drive
 * Compatible with ros2_control hardware interface “diffdrive_arduino”.
 */
#include <Arduino.h>
#include <Encoder.h>

/* ───── Pin map ─────────────────────────────────────────── */
// wheels: 0-FL  1-RL  2-FR  3-RR
const uint8_t PWM_PIN[4] = { 6, 22,  8, 24 };
const uint8_t DIR_PIN[4] = { 7, 23,  9, 25 };
const bool    DIR_POL[4] = { 0, 0, 1, 1 };   // set 1 to invert direction

/* Quadrature encoders (A,B pins) */
const uint8_t ENC_A[4]  = { 2, 4,  32, 34 };
const uint8_t ENC_B[4]  = { 3, 5,  33, 35 };
Encoder encFL(ENC_A[0], ENC_B[0]);
Encoder encRL(ENC_A[1], ENC_B[1]);
Encoder encFR(ENC_A[2], ENC_B[2]);
Encoder encRR(ENC_A[3], ENC_B[3]);
Encoder* ENCODERS[4] = { &encFL, &encRL, &encFR, &encRR };

/* ───── Parameters ──────────────────────────────────────── */
constexpr uint32_t BAUD     = 115200;  // must match Jetson side
constexpr uint16_t LOOP_HZ  = 50;      // control frequency
constexpr uint16_t MAX_PWM  = 255;
constexpr int32_t  CPS_LIM  = 6000;    // safety cap (counts / s)

/* PI controller gains (shared) */
volatile float Kp = 20.0f;
volatile float Ki =  0.0f;

/* ───── Globals ─────────────────────────────────────────── */
volatile int32_t tgt[4]      = {0,0,0,0};   // target Δ-counts / cycle
int32_t  integ[4]            = {0,0,0,0};   // integral error accumulator
int32_t  lastCounts[4]       = {0,0,0,0};   // previous encoder readings

/* Utility: clamp to range */
inline int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{ return (v < lo) ? lo : (v > hi) ? hi : v; }

/* Set one wheel’s H-bridge */
inline void setMotor(uint8_t idx, int32_t cmdCounts)
{
  /* map counts-per-cycle → PWM  */
  int32_t pwmVal = map(abs(cmdCounts), 0, CPS_LIM / LOOP_HZ, 0, MAX_PWM);
  pwmVal = clamp32(pwmVal, 0, MAX_PWM);

  bool forward = (cmdCounts >= 0) ^ DIR_POL[idx];
  digitalWrite(DIR_PIN[idx], forward ? HIGH : LOW);
  analogWrite (PWM_PIN[idx], pwmVal);
}

/* Parse a line of ASCII (blocking until EOL) */
bool readline(String& out)
{
  if (!Serial.available()) return false;
  out = Serial.readStringUntil('\n');   // include CR if present
  out.trim();                           // remove CR/LF/whitespace
  return out.length() > 0;
}

/* ───── SETUP ───────────────────────────────────────────── */
void setup()
{
  for (uint8_t p : PWM_PIN) pinMode(p, OUTPUT);
  for (uint8_t p : DIR_PIN) pinMode(p, OUTPUT);
  Serial.begin(BAUD);
  while (!Serial && millis() < 3000) { }   // wait for USB host (optional)
}

/* ───── MAIN LOOP ───────────────────────────────────────── */
elapsedMillis loopTimer;
void loop()
{
  /* ---------- 1 · Handle incoming commands ---------------- */
  String line;
  if (readline(line))
  {
    char cmd = line.charAt(0);
    if (cmd == 'm')
    { // motor command: m <cFL> <cRL> <cFR> <cRR>
      int32_t v[4];
      if (sscanf(line.c_str()+1, "%ld %ld %ld %ld",
                 &v[0], &v[1], &v[2], &v[3]) == 4)
        memcpy((void*)tgt, v, sizeof(v));
    }
    else if (cmd == 'e')
    { // encoder request
      int32_t delta[4];
      for (uint8_t i=0; i<4; ++i)
      {
        int32_t now = ENCODERS[i]->read();
        delta[i] = now - lastCounts[i];
        lastCounts[i] = now;
      }
      Serial.printf("e %ld %ld %ld %ld\n", delta[0], delta[1], delta[2], delta[3]);
    }
    else if (cmd == 'p')
    { // PI gains: p <Kp> <Ki>
      float kp, ki;
      if (sscanf(line.c_str()+1, "%f %f", &kp, &ki) >= 1)
      { Kp = kp;  Ki = (ki>=0) ? ki : Ki; }
    }
  }

  /* ---------- 2 · 50 Hz PI wheel controller --------------- */
  if (loopTimer >= 1000 / LOOP_HZ)
  {
    loopTimer = 0;

    for (uint8_t i=0; i<4; ++i)
    {
      int32_t now   = ENCODERS[i]->read();
      int32_t dCnt  = now - lastCounts[i];
      lastCounts[i] = now;

      /* PI control on Δ-counts */
      int32_t err   = tgt[i] - dCnt;
      integ[i]     += err;

      int32_t cmd   = tgt[i] + (int32_t)(Kp * err) + (int32_t)(Ki * integ[i]);
      cmd           = clamp32(cmd, -CPS_LIM/LOOP_HZ, CPS_LIM/LOOP_HZ);
      setMotor(i, cmd);
    }
  }
}
