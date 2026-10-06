// motor_beat.ino  --  TWO-motor beat on the classic ESP32.
//
// The host program (motor_beat.py) is the 100 BPM, 4/4 metronome and sends
// timed commands. This firmware just executes them:
//
//   MOTOR 2  (the "hold" motor -- the old single-motor beat behavior):
//     'C' = BEAT CLOSE -> drive to 110 deg and actively hold (closed-loop).
//     'J' = KEY CLOSE  -> drive to 110 deg and actively hold (closed-loop).
//     'O' = OPEN  -> actively return to 0 deg at HALF speed, then release (loose).
//     Host sends C on beats 2 & 4, O on beats 1 & 3  -> Motor 2 hits on 2 & 4.
//
//   MOTOR 1  (the "kick" motor, open-loop):
//     'K' = KICK  -> spin clockwise for 0.05 s, then release (loose).
//     Host sends K on EVERY quarter note (all 4 beats) -> Motor 1 hits twice
//     as often as Motor 2.
//
//   'H' = heartbeat.   'S' = stop: cut power to BOTH motors.
//   If the host goes silent (Ctrl-C / crash / unplug), the watchdog cuts power
//   to BOTH motors within HEARTBEAT_WATCHDOG_MS.
//
// Pins (classic ESP32):
//   MOTOR 1 BTS7960:  RPWM=16 LPWM=17 R_EN=18 L_EN=19   (no encoder needed)
//   MOTOR 2 BTS7960:  RPWM=25 LPWM=26 R_EN=27 L_EN=32
//   MOTOR 2 encoder:  A=33  B=34   (34 is input-only, no pull-up -> plain INPUT)
//   Encoders powered from 3.3V.  BTS7960s need their B+/B- motor supply.

#include <math.h>

// ===== Motor 1 (kick) pins =====
static const int M1_RPWM = 16;
static const int M1_LPWM = 17;
static const int M1_R_EN = 18;
static const int M1_L_EN = 19;

// ===== Motor 2 (hold) pins =====
static const int M2_RPWM = 25;
static const int M2_LPWM = 26;
static const int M2_R_EN = 27;
static const int M2_L_EN = 32;
static const int M2_ENC_A = 33;
static const int M2_ENC_B = 34;   // input-only, no pull-up

static const int PWM_FREQ = 20000;
static const int PWM_BITS = 8;
static const unsigned long SERIAL_BAUD = 115200;

// ===== Motor 1 kick =====
static const int          KICK_DUTY = 180;   // drive strength for the kick
static const unsigned long KICK_MS  = 50;    // 0.05 s

// ===== Motor 2 direction fix-ups =====
//  From the motor2_kick test: positive count -> leave INVERT_ENCODER2 = 0.
//  Negative count -> set INVERT_ENCODER2 = 1. (Wrong value => runaway on close.)
static const int INVERT_ENCODER2 = 1;   // motor2_kick: CW drive -> negative counts
static const int INVERT_MOTOR2   = 0;

// ===== Motor 2 target & control =====
static const float COUNTS_PER_MOTOR_REV  = 28.0f;
static const float GEAR_RATIO            = 19.2f;
static const float COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;
static const float BEAT_TARGET_DEGREES   = 110.0f;
static const float KEY_TARGET_DEGREES    = 110.0f;
static const long  BEAT_TARGET_COUNTS =
    (long)lroundf(COUNTS_PER_OUTPUT_REV * (BEAT_TARGET_DEGREES / 360.0f));
static const long  KEY_TARGET_COUNTS =
    (long)lroundf(COUNTS_PER_OUTPUT_REV * (KEY_TARGET_DEGREES / 360.0f));

static const unsigned long CONTROL_PERIOD_US = 5000;   // 200 Hz
static const int   MAX_DUTY = 255;
static const int   MIN_DUTY = 18;
static const float RETURN_SPEED_FRACTION = 0.5f;       // return to 0 at half speed

static float Kp = 20.0f;
static float Ki = 10.0f;
static float Kd = 0.2f;
static const float INTEGRAL_ACTIVE_COUNTS = 30.0f;
static const float INTEGRAL_TERM_LIMIT    = (float)MAX_DUTY;
static const long  REACHED_DEADBAND = 2;
static const unsigned long HEARTBEAT_WATCHDOG_MS = 400;
static const long SAFETY_LIMIT_COUNTS =
    (BEAT_TARGET_COUNTS > 0 ? BEAT_TARGET_COUNTS : 1) * 4 + 200;

// ===== Encoder 2 =====
volatile long    enc2Raw  = 0;
volatile uint8_t enc2Prev = 0;
static const int8_t QUAD[16] = {
    0, -1,  1,  0,  1,  0,  0, -1,
   -1,  0,  0,  1,  0,  1, -1,  0
};
void IRAM_ATTR onEnc2() {
  uint8_t a = (uint8_t)digitalRead(M2_ENC_A);
  uint8_t b = (uint8_t)digitalRead(M2_ENC_B);
  uint8_t s = (uint8_t)((a << 1) | b);
  enc2Raw += QUAD[(enc2Prev << 2) | s];
  enc2Prev = s;
}
long readPos2() {
  noInterrupts(); long r = enc2Raw; interrupts();
  return INVERT_ENCODER2 ? -r : r;
}

// ===== Motor drive =====
void coast1() {
  ledcWrite(M1_RPWM, 0);
  digitalWrite(M1_LPWM, LOW);
  digitalWrite(M1_R_EN, LOW);
  digitalWrite(M1_L_EN, LOW);
}
void coast2() {
  ledcWrite(M2_RPWM, 0);
  ledcWrite(M2_LPWM, 0);
  digitalWrite(M2_R_EN, LOW);
  digitalWrite(M2_L_EN, LOW);
}
void driveMotor2(float u) {
  int dir = (u >= 0.0f) ? 1 : -1;
  if (INVERT_MOTOR2) dir = -dir;
  int duty = (int)fabsf(u);
  if (duty > MAX_DUTY) duty = MAX_DUTY;
  digitalWrite(M2_R_EN, HIGH);
  digitalWrite(M2_L_EN, HIGH);
  if (duty < MIN_DUTY) { ledcWrite(M2_RPWM, 0); ledcWrite(M2_LPWM, 0); return; }
  if (dir > 0) { ledcWrite(M2_LPWM, 0); ledcWrite(M2_RPWM, duty); }
  else         { ledcWrite(M2_RPWM, 0); ledcWrite(M2_LPWM, duty); }
}

// ===== State =====
long          setpoint2   = 0;      // 0, KEY_TARGET_COUNTS, or BEAT_TARGET_COUNTS
bool          released2   = true;   // Motor 2 loose/disabled
float         integral2   = 0.0f;   // close-hold effort (persists across beats)
float         prevError2  = 0.0f;
unsigned long lastControlUs = 0;
unsigned long lastRxMs      = 0;
bool          faulted2    = false;
bool          kick1Active = false;
unsigned long kick1StartMs = 0;

void startKick1() {
  digitalWrite(M1_R_EN, HIGH);
  digitalWrite(M1_L_EN, HIGH);
  digitalWrite(M1_LPWM, LOW);
  ledcWrite(M1_RPWM, KICK_DUTY);   // clockwise
  kick1Active = true;
  kick1StartMs = millis();
}
void m2Open()  { setpoint2 = 0;            released2 = false; prevError2 = (float)(0 - readPos2()); }
void m2Close(long targetCounts) {
  setpoint2 = targetCounts;
  released2 = false;
  faulted2 = false;
  prevError2 = (float)(targetCounts - readPos2());
}
void releaseAll() {
  coast1(); kick1Active = false;
  coast2(); released2 = true; setpoint2 = 0;
}

void setup() {
  // Motor 1
  pinMode(M1_R_EN, OUTPUT); pinMode(M1_L_EN, OUTPUT);
  pinMode(M1_LPWM, OUTPUT); digitalWrite(M1_LPWM, LOW);
  ledcAttach(M1_RPWM, PWM_FREQ, PWM_BITS);
  // Motor 2
  pinMode(M2_R_EN, OUTPUT); pinMode(M2_L_EN, OUTPUT);
  ledcAttach(M2_RPWM, PWM_FREQ, PWM_BITS);
  ledcAttach(M2_LPWM, PWM_FREQ, PWM_BITS);
  pinMode(M2_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_B, INPUT);          // GPIO34: input-only, no pull-up

  coast1(); coast2();

  noInterrupts();
  enc2Prev = (uint8_t)((digitalRead(M2_ENC_A) << 1) | digitalRead(M2_ENC_B));
  enc2Raw = 0;
  interrupts();
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), onEnc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_B), onEnc2, CHANGE);

  Serial.begin(SERIAL_BAUD);
  Serial.println("READY motor_beat (2 motors)");
  Serial.print("Motor2 targets: keys ");
  Serial.print(KEY_TARGET_DEGREES, 0);
  Serial.print(" deg=");
  Serial.print(KEY_TARGET_COUNTS);
  Serial.print(" counts, beat ");
  Serial.print(BEAT_TARGET_DEGREES, 0);
  Serial.print(" deg=");
  Serial.print(BEAT_TARGET_COUNTS);
  Serial.println(" counts");
  Serial.print("Cmds: K=motor1 kick  J=motor2 ");
  Serial.print(KEY_TARGET_DEGREES, 0);
  Serial.print("  C=motor2 ");
  Serial.print(BEAT_TARGET_DEGREES, 0);
  Serial.println("  O=motor2 open  H=beat  S=stop(all)");

  lastRxMs = millis();
  lastControlUs = micros();
  releaseAll();
}

void loop() {
  // ---- Serial ----
  while (Serial.available() > 0) {
    const int c = Serial.read();
    lastRxMs = millis();
    if      (c == 'K' || c == 'k') startKick1();
    else if (c == 'C' || c == 'c') m2Close(BEAT_TARGET_COUNTS);
    else if (c == 'J' || c == 'j') m2Close(KEY_TARGET_COUNTS);
    else if (c == 'O' || c == 'o') m2Open();
    else if (c == 'S' || c == 's') { releaseAll(); Serial.println("STOP -- all motors released."); }
  }

  // ---- Motor 1 kick timing (non-blocking; runs alongside Motor 2) ----
  if (kick1Active && (millis() - kick1StartMs >= KICK_MS)) {
    coast1();
    kick1Active = false;
  }

  // ---- Watchdog: host silent -> cut power to BOTH motors ----
  if ((millis() - lastRxMs) > HEARTBEAT_WATCHDOG_MS) {
    if (!released2 || kick1Active) {
      releaseAll();
      Serial.println("LINK LOST -- all motors released.");
    }
  }

  // ---- Motor 2 closed-loop control (rate-limited) ----
  const unsigned long nowUs = micros();
  const unsigned long dtUs = nowUs - lastControlUs;
  if (dtUs < CONTROL_PERIOD_US) return;
  lastControlUs = nowUs;
  const float dt = dtUs / 1000000.0f;

  const long position = readPos2();

  if (!faulted2 && labs(position) > SAFETY_LIMIT_COUNTS) {
    faulted2 = true;
    coast2();
    Serial.print("FAULT: Motor 2 runaway, pos=");
    Serial.print(position);
    Serial.println(" -> check INVERT_ENCODER2.");
  }
  if (faulted2) { coast2(); return; }

  if (released2) { coast2(); return; }

  // Reached 0 on the return -> latch loose (stops oscillation at 0).
  if (setpoint2 == 0 && labs(position) <= REACHED_DEADBAND) {
    released2 = true;
    coast2();
    return;
  }

  const float error = (float)(setpoint2 - position);
  const bool  closing = (setpoint2 != 0);

  // Integral = close-hold effort; accumulate/apply only while closing so it
  // never fights the return, and persist it across beats for a full close.
  if (closing && fabsf(error) <= INTEGRAL_ACTIVE_COUNTS) {
    integral2 += error * dt;
    const float maxI = INTEGRAL_TERM_LIMIT / (Ki > 0.0f ? Ki : 1.0f);
    if (integral2 >  maxI) integral2 =  maxI;
    if (integral2 < -maxI) integral2 = -maxI;
  }
  const float iTerm = closing ? (Ki * integral2) : 0.0f;

  const float derivative = (error - prevError2) / dt;
  prevError2 = error;

  const float dutyCap = closing ? (float)MAX_DUTY
                                : (float)MAX_DUTY * RETURN_SPEED_FRACTION;

  float u = Kp * error + iTerm + Kd * derivative;
  if (u >  dutyCap) u =  dutyCap;
  if (u < -dutyCap) u = -dutyCap;

  driveMotor2(u);
}
