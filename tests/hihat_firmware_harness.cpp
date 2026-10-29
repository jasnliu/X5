// Run the actual firmware parser/control flow without an ESP32 or any motor.
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <deque>
#include <iostream>
#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define CHANGE 3
unsigned long clockMs = 100;
unsigned long millis() { return clockMs; }
unsigned long micros() { return clockMs*1000; }
void noInterrupts() {} void interrupts() {}
int digitalLevels[40] = {}, pwmLevels[40] = {};
int pwmFrequencies[40] = {}, pwmBits[40] = {};
void pinMode(int,int) {}
void digitalWrite(int pin,int level) { digitalLevels[pin] = level; }
int digitalRead(int) { return 0; }
bool ledcAttach(int pin,int frequency,int bits) {
  pwmFrequencies[pin] = frequency; pwmBits[pin] = bits; return true;
}
void ledcWrite(int pin,int duty) { pwmLevels[pin] = duty; }
int digitalPinToInterrupt(int p) { return p; }
void attachInterrupt(int,void(*)(),int) {}
struct SerialMock {
  std::deque<char> input; std::string output;
  void begin(int) {} int available() { return input.size(); }
  int read() { int c=input.front(); input.pop_front(); return c; }
  void print(const char* s) { output += s; }
  template<class T> void print(T v) { output += std::to_string(v); }
  void print(float v,int) { output += std::to_string(v); }
  void println(const char* s) { output += s; output += '\n'; }
  void printf(const char* f, ...) {
    char buffer[512]; va_list args; va_start(args,f);
    vsnprintf(buffer,sizeof(buffer),f,args);va_end(args);output+=buffer;
  }
} Serial;
#include "../esp32_hihat/motor_beat/motor_beat.ino"
void send(const std::string& s) {
  for (char c:s) Serial.input.push_back(c);
  clockMs += 10; loop();
}
void pos(long p) { enc2Raw = -p; }
void assertKickReleased() {
  assert(!kick1Active && pwmLevels[M1_RPWM]==0 && digitalLevels[M1_LPWM]==LOW);
  assert(digitalLevels[M1_R_EN]==LOW && digitalLevels[M1_L_EN]==LOW);
}
void testBassPulse() {
  assert(KICK_DUTY==180 && KICK_MS==50);
  assert(M1_RPWM==16 && M1_LPWM==17 && M1_R_EN==18 && M1_L_EN==19);
  assert(pwmFrequencies[M1_RPWM]==20000 && pwmBits[M1_RPWM]==8);
  assertKickReleased();
  send("J"); pos(KEY_TARGET_COUNTS); send("H");
  send("K");
  assert(kick1Active && pwmLevels[M1_RPWM]==180 && digitalLevels[M1_LPWM]==LOW);
  assert(digitalLevels[M1_R_EN]==HIGH && digitalLevels[M1_L_EN]==HIGH);
  clockMs+=49; loop();
  assert(kick1Active && pwmLevels[M1_RPWM]==180);
  assert(!released2 && setpoint2==KEY_TARGET_COUNTS);
  clockMs+=1; loop(); assertKickReleased();
  assert(!released2 && setpoint2==KEY_TARGET_COUNTS); // Bass release never opens hi-hat.
  clockMs+=100; loop(); assertKickReleased(); // No autonomous repeat or reverse.
  send("K"); assert(kick1Active); // Next host beat starts a fresh identical pulse.
  send("S"); assertKickReleased(); assert(released2); // Emergency stop cuts pulse short.
  send("O"); pos(0); send("H"); assert(released2 && openReached2);
  std::cout << "Bass clockwise PWM 180/255 for 50 ms, release and independent hi-hat passed\n";
}
int main() {
  setup(); testBassPulse(); send("Q");
  assert(Serial.output.find("HIHAT v=2 min=90 max=115") != std::string::npos);
  send("A115\n"); assert(calibratedDegrees==115 && calibratedCounts==172);
  assert(released2 && setpoint2==0);  // Setting an angle never moves the motor.
  send("B"); assert(setpoint2==172 && !released2);
  send("A95\n"); assert(calibratedDegrees==115); // Can't retarget while closed.
  pos(172); send("H"); assert(!movementPending);
  send("C"); assert(setpoint2==149); // Legacy collector remains fixed at 100.
  send("J"); assert(setpoint2==149);
  send("O"); pos(0); send("H"); assert(released2);
  assert(openReached2); pos(-19); send("Q");
  assert(openReached2 && released2); // Passive coast does not erase return proof.
  send("A110\n"); assert(calibratedDegrees==110); // Allowed after actual zero return.
  send("A115\n");
  for (const auto& s:{"A116\n","A120\n","A89\n","A-90\n","A90.5\n","A999999999\n","A\n"}) {
    send(s); assert(calibratedDegrees==115 && released2 && setpoint2==0);
  }
  send("A90BK\n"); assert(!kick1Active && released2 && calibratedDegrees==115);
  send("A11"); clockMs+=160; send("H"); assert(!readingAngle && released2);
  send("A90\n"); assert(calibratedDegrees==90 && calibratedCounts==134);
  send("A9S"); assert(!readingAngle && released2); // Stop interrupts partial frames.
  pos(0);send("V");
  assert(Serial.output.find("COLLECTION v=1 min=1 max=90 guard=100")!=std::string::npos);
  for (const auto& s:{"P0\n","P91\n","P-20\n","P20.5\n","P80BK\n","P999999999\n","P\n"}) {
    send(s); assert(released2 && setpoint2==0 && !kick1Active);
  }
  send("P5S");assert(released2 && !readingAngle);
  send("P55\n");assert(setpoint2==82 && partialCollectionActive && !released2);
  assert(calibratedDegrees==90 && calibratedCounts==134); // Normal B target unchanged.
  pos(82);send("H");assert(!movementPending);
  send("P60\n");assert(setpoint2==82); // Retarget requires an actual open return.
  for(int n=0;n<30;n++){clockMs+=100;send("H");}
  assert(!faulted2 && !released2); // Sustained shallow hold is allowed.
  send("O");pos(0);send("H");assert(released2 && openReached2 && !partialCollectionActive);
  send("P30\n");clockMs+=401;loop();assert(released2 && !partialCollectionActive);
  send("O");pos(0);send("H");
  send("B");
  assert(!openReached2);
  for(int n=0;n<22;n++) { clockMs+=100;send("H"); }
  assert(faulted2 && released2 && setpoint2==0); // Fresh heartbeats don't mask stall.
  send("B");send("C");send("J");send("O");send("S");
  assert(faulted2 && released2 && setpoint2==0); // Fault can't be cleared by a beat.
  send("P30\n");assert(faulted2 && released2);
  // Simulated reset only: prove the collection-specific measured travel guard.
  faulted2=false;pos(0);send("O");send("H");send("P60\n");
  pos(COLLECTION_TRAVEL_LIMIT_COUNTS+1);send("H");
  assert(faulted2 && released2 && !kick1Active && setpoint2==0);
  std::cout << "Firmware parser, 115 degree cap, legacy targets, encoder status and stall latch passed\n";
}
