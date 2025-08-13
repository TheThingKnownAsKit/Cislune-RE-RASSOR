/*
 * RE-RASSOR Teensy 4.1 firmware — two-channel diff-drive
 * Contract for ros2_control:
 *   - Command:  M <dL> <dR>   (target change in-encoder-counts per control cycle, left/right)
 *   - Feedback: E <dL> <dR>   (measured change in-encoder-counts since last E, averaged per side)
 *   - Gains:    p <Kp> <Ki>
 *
 * Electrical:
 *   - RC-style ESC pulses on PWM pins (1000–2000 us; 1500 us neutral)
 *   - Quadrature encoders on A/B pins per wheel
 *
 * Wheel indexing inside firmware (fixed):
 *   0 = FL, 1 = RL, 2 = FR, 3 = RR
 */

#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>

/* ───── Pin map per your table ─────────────────────────────
   RR (Back Right): PWM 6,  ESC TX->RX2 (7),  ESC RX->TX2 (8)
   RL (Back Left) : PWM 5,  ESC TX->RX3 (15), ESC RX->TX3 (14)
   FR (Front Right): PWM 4, ESC TX->RX4 (16), ESC RX->TX4 (17)
   FL (Front Left): PWM 3,  ESC TX->RX5 (21), ESC RX->TX5 (20)

   Firmware order: [FL, RL, FR, RR] → PWM pins {3, 5, 4, 6}
*/
const uint8_t PWM_PIN[4] = { 3, 5, 4, 6 };

// Digital pins for encoder interrupts
const uint8_t ENC_A[4] = { 30, 28, 32, 34 };    // FL, RL, FR, RR
const uint8_t ENC_B[4] = { 31, 29, 33, 35 };    // FL, RL, FR, RR

Encoder encFL(ENC_A[0], ENC_B[0]);
Encoder encRL(ENC_A[1], ENC_B[1]);
Encoder encFR(ENC_A[2], ENC_B[2]);
Encoder encRR(ENC_A[3], ENC_B[3]);
Encoder* ENCODERS[4] = { &encFL, &encRL, &encFR, &encRR };

// Initialize 4 ESC Servo objects for wheels
Servo esc[4];



/* ───── Parameters ───────────────────────────────────────── */
constexpr uint32_t BAUD = 115200;
constexpr uint16_t LOOP_HZ   = 50;
constexpr int32_t  CPS_LIM   = 6000;            // safety cap (counts/s)

// PI controller gains (shared)
volatile float Kp = 20.0f;
volatile float Ki =  0.0f;



/* ───── State ────────────────────────────────────────────── */
volatile int32_t tgt_side[2] = {0, 0};          // target change in counts/cycle: [Left, Right]
int32_t  tgt[4]              = {0, 0, 0, 0};    // expanded per wheel each cycle
int32_t  integ[4]            = {0, 0, 0, 0};    // PI integrators per wheel
int32_t  lastCountsCtl[4]    = {0, 0, 0, 0};    // encoder baselines for control
int32_t  lastCountsRpt[4]    = {0, 0, 0, 0};    // encoder baselines for E reporting (independent)



/* ───── Utilities ───────────────────────────────────────── */

// Returns the value v rounded to either lo or hi
inline int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Converts desired velocity (in counts per cycle) to PWM micro seconds
// Converts to RC servo pulses of 1000-2000
inline int16_t countsToMicros(int32_t cmdCountsPerCycle) {
  const int32_t scale = CPS_LIM / LOOP_HZ;    // e.g., 6000/50 = 120

  // Ensure that the desired counts/cycle is within the limit
  int32_t clamped_counts = clamp32(cmdCountsPerCycle, -scale, scale);

  // Calculate the PWM microseconds corresponding to velocity command
  int32_t pwm_ms = 1500 + (clamped_counts * 500) / scale;

  return (int16_t)clamp32(pwm_ms, 1000, 2000);
}

inline void setMotor(uint8_t idx, int32_t cmdCounts) {
  esc[idx].writeMicroseconds(countsToMicros(cmdCounts));
}

// Reads from the Serial connection until newline is reached
bool readline(String& out) {
  if (!Serial.available()) return false;

  out = Serial.readStringUntil('\n');
  out.trim();

  return out.length() > 0;
}



/* ───── Setup ────────────────────────────────────────────── */
void setup() {
  Serial.begin(BAUD);
  while (!Serial && millis() < 3000) { }        // wait for USB host

  for (uint8_t i=0; i<4; ++i) {
    esc[i].attach(PWM_PIN[i], 1000, 2000);
    esc[i].writeMicroseconds(1500);             // motors are neutral at setup
  }

  // Initialize encoder baselines for both control and reporting
  for (uint8_t i=0; i<4; ++i) {
    int32_t now = ENCODERS[i]->read();
    lastCountsCtl[i] = now;
    lastCountsRpt[i] = now;
  }
}



/* ───── Main loop ───────────────────────────────────────── */
elapsedMillis loopTimer;

void loop() {
  // 1. Parse commands from Serial communication (sent by diffdrive_arduino)
  String line;
  if (readline(line)) {
    char cmd = line.charAt(0);

    if (cmd == 'M') {
      // 2-channel command: M <left_dcounts/cycle> <right_dcounts/cycle>
      int32_t L, R;
      if (sscanf(line.c_str()+1, "%ld %ld", &L, &R) == 2) {
        tgt_side[0] = L;                        // Left side target (change in counts/cycle)
        tgt_side[1] = R;                        // Right side target (change in counts/cycle)
      }
    }
    else if (cmd == 'E') {
      // Encoder feedback: average per side since last E (independent baseline)
      int32_t now[4], d[4];
      for (uint8_t i=0; i<4; ++i) {
        now[i] = ENCODERS[i]->read();
        d[i]   = now[i] - lastCountsRpt[i];
        lastCountsRpt[i] = now[i];
      }
      int32_t dLeft  = (d[0] + d[1]) / 2;       // FL + RL
      int32_t dRight = (d[2] + d[3]) / 2;       // FR + RR
      Serial.printf("E %ld %ld\n", dLeft, dRight);
    }
    else if (cmd == 'p') {
      // PI gains: p <Kp> <Ki>
      float kp, ki;
      if (sscanf(line.c_str()+1, "%f %f", &kp, &ki) >= 1) {
        Kp = kp;
        if (ki >= 0) Ki = ki;
      }
    }
  }

  // 2. Send appropriate wheel PWM commands every LOOP_HZ cycle
  if (loopTimer >= 1000 / LOOP_HZ) {
    loopTimer = 0;

    // Expand side targets to per-wheel targets each cycle
    // Left side = FL, RL ; Right side = FR, RR
    tgt[0] = tgt[1] = tgt_side[0];              // FL, RL
    tgt[2] = tgt[3] = tgt_side[1];              // FR, RR

    for (uint8_t i=0; i<4; ++i)
    {
      int32_t now  = ENCODERS[i]->read();
      int32_t dCnt = now - lastCountsCtl[i];    // measured change in counts this cycle
      lastCountsCtl[i] = now;

      int32_t err  = tgt[i] - dCnt;
      integ[i]    += err;

      int32_t cmd = tgt[i]
                  + (int32_t)(Kp * err)
                  + (int32_t)(Ki * integ[i]);

      // Saturate per-cycle command and send to ESC
      cmd = clamp32(cmd, -CPS_LIM/LOOP_HZ, CPS_LIM/LOOP_HZ);
      setMotor(i, cmd);
    }
  }
}
