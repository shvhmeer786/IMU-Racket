/*
 * I2C Scanner for Arduino
 * This sketch scans for I2C devices and reports their addresses
 * Upload this to your Arduino to check if the IMU is detected
 */

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("I2C Scanner starting...");
  Serial.println("Scanning for I2C devices...");
  
  Wire.begin();
}

void loop() {
  byte error, address;
  int deviceCount = 0;

  Serial.println("Scanning I2C bus...");
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      deviceCount++;
      
      // Common IMU addresses
      if (address == 0x6A || address == 0x6B) {
        Serial.println("  -> This looks like an LSM6DS33 IMU!");
      }
      if (address == 0x68 || address == 0x69) {
        Serial.println("  -> This looks like an MPU6050/MPU9250 IMU!");
      }
      if (address == 0x1C || address == 0x1D) {
        Serial.println("  -> This looks like an LSM303 accelerometer!");
      }
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("Check your wiring:");
    Serial.println("  - SDA and SCL connections");
    Serial.println("  - Power connections (3.3V or 5V)");
    Serial.println("  - Pull-up resistors (usually built-in)");
  }
  else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }
  
  Serial.println();
  delay(5000);  // Wait 5 seconds before next scan
} 