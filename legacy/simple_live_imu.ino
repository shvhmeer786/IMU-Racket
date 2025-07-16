/*
 * Simple Live IMU for Arduino Nano RP2040 Connect
 * 
 * This sketch sends continuous real-time IMU orientation data
 * Uses basic approach for maximum compatibility
 * 
 * Upload this to your Arduino using Arduino IDE
 */

#include <Arduino_LSM6DSOX.h>
#include <math.h>

// Timing for consistent data rate
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100; // 100ms = 10Hz

// Simple orientation tracking
float heading = 0.0;
float roll = 0.0;
float pitch = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection (up to 3 seconds)
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime) < 3000) {
    delay(10);
  }
  
  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      delay(1000);
    }
  }
  
  // Startup messages
  Serial.println("Simple Live IMU v1.0");
  Serial.println("Sending continuous orientation data...");
  Serial.println("Move the Arduino to see real-time changes!");
  
  delay(1000); // Brief pause for stability
}

void loop() {
  unsigned long currentTime = millis();
  
  // Send data at consistent intervals
  if (currentTime - lastUpdate >= updateInterval) {
    lastUpdate = currentTime;
    
    // Read accelerometer and gyroscope
    float ax, ay, az;
    float gx, gy, gz;
    
    if (IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      // Calculate simple orientation from accelerometer
      pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
      roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
      
      // Simple gyroscope integration for heading
      heading += gz * 0.1;
      if (heading > 360) heading -= 360;
      if (heading < 0) heading += 360;
      
      // Send orientation data in clean format
      Serial.print("Orientation: ");
      Serial.print(heading, 2);
      Serial.print(", ");
      Serial.print(roll, 2);
      Serial.print(", ");
      Serial.println(pitch, 2);
      
      // Send accelerometer data
      Serial.print("Accel: ");
      Serial.print(ax, 2);
      Serial.print(", ");
      Serial.print(ay, 2);
      Serial.print(", ");
      Serial.println(az, 2);
      
      // Send gyroscope data
      Serial.print("Gyro: ");
      Serial.print(gx, 2);
      Serial.print(", ");
      Serial.print(gy, 2);
      Serial.print(", ");
      Serial.println(gz, 2);
    } else {
      Serial.println("Failed to read IMU data");
    }
  }
}

// Alternative: If IMU library doesn't work, we can use fake data for testing
void sendFakeData() {
  static float counter = 0;
  counter += 0.1;
  
  // Generate smooth fake orientation changes
  float fake_heading = 180 + 60 * sin(counter * 0.1);
  float fake_roll = 30 * sin(counter * 0.15);
  float fake_pitch = 20 * cos(counter * 0.12);
  
  Serial.print("Orientation: ");
  Serial.print(fake_heading, 2);
  Serial.print(", ");
  Serial.print(fake_roll, 2);
  Serial.print(", ");
  Serial.println(fake_pitch, 2);
  
  Serial.print("Accel: ");
  Serial.print(0.1 * sin(counter), 2);
  Serial.print(", ");
  Serial.print(0.1 * cos(counter), 2);
  Serial.print(", ");
  Serial.println(9.81, 2);
  
  Serial.print("Gyro: ");
  Serial.print(5 * sin(counter * 0.2), 2);
  Serial.print(", ");
  Serial.print(5 * cos(counter * 0.2), 2);
  Serial.print(", ");
  Serial.println(2 * sin(counter * 0.3), 2);
} 