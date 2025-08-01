/* Teensy firmware for diffdrive_arduino + ros_arduino_bridge
 * Drives TWO motors per side (total four) that share the same command.
 */
#include <Encoder.h>

/* -------- Pin map -------- */
const uint8_t PWM_L[2] = {6, 22};
const uint8_t DIR_L[2] = {7, 23};
const uint8_t PWM_R[2] = {8, 24};
const uint8_t DIR_R[2] = {9, 25};
const uint8_t ENC_L_A = 2, ENC_L_B = 3;
const uint8_t ENC_R_A = 4, ENC_R_B = 5;

/* -------- Parameters ----- */
constexpr uint32_t BAUD      = 115200;
constexpr uint16_t LOOP_HZ   = 50;            // must match loop_rate on Jetson
constexpr uint16_t MAX_PWM   = 255;
constexpr int32_t  CPS_LIMIT = 6000;          // encoder counts/s safety cap

/* PID terms (tweak via "p" command) */
float KP = 20, KI = 0;
float integL = 0, integR = 0;

/* -------- Globals -------- */
Encoder encL(ENC_L_A, ENC_L_B), encR(ENC_R_A, ENC_R_B);
volatile int32_t tgtL = 0, tgtR = 0;

/* Helpers */
inline int32_t clamp32(int32_t v, int32_t lo, int32_t hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

void setSide(int32_t counts, const uint8_t pwm[2], const uint8_t dir[2])
{
  int32_t pwm_val = map(abs(counts), 0, CPS_LIMIT / LOOP_HZ, 0, MAX_PWM);
  pwm_val = clamp32(pwm_val, 0, MAX_PWM);
  bool fwd = counts >= 0;
  for (int i = 0; i < 2; ++i) {
    digitalWrite(dir[i], fwd ? HIGH : LOW);
    analogWrite(pwm[i], pwm_val);
  }
}

elapsedMillis tLoop;
void loop()
{
  /* --- 1. Serial protocol --- */
  if (Serial1.available()) {
    char c = Serial1.read();
    if (c == 'm') { tgtL = Serial1.parseInt(); tgtR = Serial1.parseInt(); }
    else if (c == 'e') {
      int32_t dL = encL.readAndReset();
      int32_t dR = encR.readAndReset();
      Serial1.printf("e %ld %ld\n", dL, dR);
    }
    else if (c == 'p') { KP = Serial1.parseFloat(); 
                         /* d & i next but d unused */ 
                         Serial1.parseFloat(); KI = Serial1.parseFloat();
                         Serial1.parseFloat(); /* o ignored */ }
  }

  /* --- 2. Control loop --- */
  if (tLoop >= 1000 / LOOP_HZ) {
    tLoop = 0;
    int32_t actL = encL.readAndReset();
    int32_t actR = encR.readAndReset();

    int32_t errL = tgtL - actL;
    int32_t errR = tgtR - actR;

    integL += errL; integR += errR;

    int32_t cmdL = tgtL + KP*errL + KI*integL;
    int32_t cmdR = tgtR + KP*errR + KI*integR;

    setSide(cmdL, PWM_L, DIR_L);
    setSide(cmdR, PWM_R, DIR_R);
  }
}

void setup()
{
  for (uint8_t pin : {PWM_L[0], PWM_L[1], PWM_R[0], PWM_R[1]}) pinMode(pin, OUTPUT);
  for (uint8_t pin : {DIR_L[0], DIR_L[1], DIR_R[0], DIR_R[1]}) pinMode(pin, OUTPUT);
  Serial1.begin(BAUD);
}
