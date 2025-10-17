#include "Debugger.h"

#include "Arduino.h"
#include "Wire.h"

void Debugger::scanI2C() {
  Serial.println("I2C Scanner");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
    }
  }
}