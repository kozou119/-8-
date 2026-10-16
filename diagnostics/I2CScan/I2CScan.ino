#include <Wire.h>
// Standalone diagnostic: does not initialize motors or write sensor registers.
void scan() {
  Serial.println(F("I2C scan: A4 SDA / A5 SCL. Addresses alone do not identify models."));
  unsigned count = 0;
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    Wire.beginTransmission(address);
    uint8_t status = Wire.endTransmission();
    if (status == 0) {
      Serial.print(F("ACK 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      ++count;
    } else if (status != 2) {
      Serial.print(F("Bus error at 0x")); Serial.print(address, HEX);
      Serial.print(F(" status=")); Serial.println(status);
    }
  }
  Serial.print(F("Devices: ")); Serial.println(count);
}
void setup() {
  Serial.begin(115200);
  uint32_t started = millis();
  while (!Serial && uint32_t(millis() - started) < 5000) delay(10);
  Wire.begin();
#if defined(ARDUINO_UNOR4_MINIMA)
  Wire.setWireTimeout(25000, true);
#endif
  scan();
  Serial.println(F("Send s to scan again."));
}
void loop() {
  if (Serial.available() && Serial.read() == 's') scan();
}
