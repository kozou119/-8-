// Arduino API simulation: verifies software behavior, not physical compatibility.
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include "BoardIO.h"
#include "Ultrasonic.h"
static uint32_t clockMs;
static int digitalLevels[20], pwm[20], operations, bits, triggers;
static unsigned long pulseUs, lastTimeout;
static void valid(int pin) { assert(pin>=0 && pin<20); }
void pinMode(int pin, int) { valid(pin); ++operations; }
int digitalRead(int pin) { valid(pin); return digitalLevels[pin]; }
void digitalWrite(int pin, int value) { valid(pin); digitalLevels[pin]=value; ++operations; if (value==HIGH) ++triggers; }
void analogWrite(int pin, int value) { valid(pin); assert(value>=0 && value<=255); pwm[pin]=value; ++operations; }
void analogWriteResolution(int value) { bits=value; }
bool digitalPinHasPWM(int pin) { return pin==3 || pin==5 || pin==6 || pin==9 || pin==10 || pin==11; }
unsigned long millis() { return clockMs; }
void delay(unsigned long ms) { clockMs+=ms; }
void delayMicroseconds(unsigned int) {}
unsigned long pulseIn(int pin, int state, unsigned long timeout) {
  valid(pin); assert(state==HIGH); lastTimeout=timeout;
  clockMs += pulseUs ? pulseUs/1000 : timeout/1000;
  return pulseUs;
}
int main() {
  using namespace robotio;
  const int layout[] = {10,8,9,7,12,2,4,-1}; assert(pinLayoutValid(layout,8));
  const int overlap[] = {10,8,9,7,12,2,8,-1}; assert(!pinLayoutValid(overlap,8));
  const int uart[] = {0,1}; assert(!pinLayoutValid(uart,2));
  const int i2c[] = {18,19}; assert(!pinLayoutValid(i2c,2));
  Motors absent(MotorPins(-1,-1,-1,-1));
  assert(!absent.begin()); absent.setSpeeds(400,400); assert(operations==0);
  Motors duplicate(MotorPins(9,7,9,8)); assert(!duplicate.begin()); assert(operations==0);
  Motors wrongPwm(MotorPins(2,8,9,7)); assert(!wrongPwm.begin()); assert(operations==0);
  Motors reserved(MotorPins(10,0,9,7)); assert(!reserved.begin()); assert(operations==0);
  Motors motors(MotorPins(10,8,9,7)); assert(motors.begin());
  assert(bits==8 && pwm[10]==0 && pwm[9]==0);
  motors.setSpeeds(400,-400);
  assert(pwm[10]==255 && digitalLevels[8]==LOW);
  assert(pwm[9]==255 && digitalLevels[7]==HIGH);
  motors.setSpeeds(INT_MIN,INT_MAX);
  assert(pwm[10]==255 && pwm[9]==255 && digitalLevels[8]==HIGH && digitalLevels[7]==LOW);
  motors.setSpeeds(110,0); assert(pwm[10]==70 && pwm[9]==0);
  Motors flipped(MotorPins(10,8,9,7,true,false)); assert(flipped.begin());
  flipped.setSpeeds(400,400); assert(digitalLevels[8]==HIGH && digitalLevels[7]==LOW);
  Button absentButton(-1); assert(!absentButton.begin()); assert(!absentButton.getSingleDebouncedPress());
  digitalLevels[12]=HIGH; Button button(12); assert(button.begin());
  clockMs=70000; assert(!button.getSingleDebouncedPress());
  digitalLevels[12]=LOW; assert(!button.getSingleDebouncedPress());
  clockMs+=19; assert(!button.getSingleDebouncedPress());
  clockMs+=1; assert(button.getSingleDebouncedPress()); assert(!button.getSingleDebouncedPress());
  digitalLevels[12]=HIGH; button.getSingleDebouncedPress();
  clockMs+=20; assert(!button.getSingleDebouncedPress());
  clockMs=UINT32_MAX-10; digitalLevels[12]=LOW;
  assert(!button.getSingleDebouncedPress()); clockMs+=20;
  assert(button.getSingleDebouncedPress()); // Correct across millis() wrap.
  TriggerEchoRanger absentRange(-1,-1); assert(!absentRange.begin());
  int mm=0; uint32_t at=0; assert(!absentRange.read(mm,at));
  TriggerEchoRanger range(2,4); assert(range.begin());
  digitalLevels[4]=LOW; pulseUs=2000; clockMs=1000;
  assert(range.read(mm,at)); assert(mm==343 && at==1002 && lastTimeout==12000);
  int before=triggers; clockMs+=20; pulseUs=3000;
  assert(range.read(mm,at)); assert(mm==343 && at==1002 && triggers==before); // Cached observation keeps original age.
  clockMs+=60; assert(range.read(mm,at)); assert(mm==514 && at==1085);
  clockMs+=60; digitalLevels[4]=HIGH; assert(!range.read(mm,at));
  digitalLevels[4]=LOW; pulseUs=0; assert(range.read(mm,at)); assert(mm==-1);
  clockMs+=60; pulseUs=50; assert(!range.read(mm,at));
  puts("PASS: simulated UNO R4 pin guards, PWM scaling, button debounce/wrap, ultrasonic timing/cache");
}
