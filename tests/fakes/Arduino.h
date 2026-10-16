#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H
#include <stdint.h>
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
void pinMode(int pin, int mode);
int digitalRead(int pin);
void digitalWrite(int pin, int value);
void analogWrite(int pin, int value);
void analogWriteResolution(int bits);
bool digitalPinHasPWM(int pin);
unsigned long millis();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(int pin, int state, unsigned long timeout);
#endif
