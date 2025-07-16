/*
 * Robust IMU for Arduino Nano RP2040 Connect
 * 
 * Handles IMU initialization failures gracefully
 * Falls back to test data if IMU fails
 */

#include <Arduino_LSM6DSOX.h>
#include <math.h>

// State variables
bool imuWorking = false;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100; // 100ms = 10Hz

// Test data variables
float testHeading = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Robust IMU Test v1.0");
  
  // Try to initialize IMU
  if (IMU.begin()) {
    imuWorking = true;
    Serial.println("✅ IMU initialized successfully - using REAL data");
  } else {
    imuWorking = false;
    Serial.println("❌ IMU initialization failed - using TEST data");
  }
  
  Serial.println("Starting data transmission...");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdate >= updateInterval) {
    lastUpdate = currentTime;
    
    if (imuWorking) {
      sendRealIMUData();
    } else {
      sendTestData();
    }
  }
}

void sendRealIMUData() {
  float ax, ay, az;
  float gx, gy, gz;
  
  // Try to read IMU data
  if (IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
    // Calculate simple orientation
    float pitch = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI;
    float roll = atan2(-ax, az) * 180.0 / PI;
    float yaw = 0.0; // Simple approximation
    
    // Send orientation data
    Serial.print("Orientation: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(roll, 2);
    Serial.print(", ");
    Serial.println(pitch, 2);
    
  } else {
    Serial.println("❌ Failed to read IMU data");
  }
}

void sendTestData() {
  // Generate rotating test data
  testHeading += 3.0; // Rotate 3 degrees per update
  if (testHeading >= 360.0) testHeading -= 360.0;
  
  float roll = 15.0 * sin(millis() * 0.001); // Slow roll oscillation
  float pitch = 10.0 * cos(millis() * 0.0015); // Slow pitch oscillation
  
  Serial.print("Orientation: ");
  Serial.print(testHeading, 2);
  Serial.print(", ");
  Serial.print(roll, 2);
  Serial.print(", ");
  Serial.println(pitch, 2);
} 