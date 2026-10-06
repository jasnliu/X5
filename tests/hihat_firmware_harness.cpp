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
void pinMode(int,int) {} void digitalWrite(int,int) {}
int digitalRead(int) { return 0; }
bool ledcAttach(int,int,int) { return true; }
void ledcWrite(int,int) {}
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
int main() {
  setup(); send("Q");
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
  send("B");
  assert(!openReached2);
  for(int n=0;n<22;n++) { clockMs+=100;send("H"); }
  assert(faulted2 && released2 && setpoint2==0); // Fresh heartbeats don't mask stall.
  send("B");send("C");send("J");send("O");send("S");
  assert(faulted2 && released2 && setpoint2==0); // Fault can't be cleared by a beat.
  std::cout << "Firmware parser, 115 degree cap, legacy targets, encoder status and stall latch passed\n";
}
