/*
 * Live IMU Data Sender for Arduino Nano RP2040 Connect
 * 
 * This sketch sends continuous real-time IMU orientation data
 * Perfect for live motion tracking and visualization
 * 
 * Upload this to your Arduino using Arduino IDE
 */

#include <Arduino_LSM6DSOX.h>
#include <math.h>

// Timing for consistent data rate
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100; // 100ms = 10Hz

// Complementary filter for orientation
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

// Filter coefficient (0.02 = 2% gyro, 98% accel)
const float alpha = 0.02;

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
    while (1);
  }
  
  // Startup messages
  Serial.println("Live IMU Data Sender v1.0");
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
      // Calculate orientation from accelerometer (tilt sensing)
      float accel_pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
      float accel_roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
      
      // Simple complementary filter
      pitch = alpha * (pitch + gy * 0.1) + (1 - alpha) * accel_pitch;
      roll = alpha * (roll + gx * 0.1) + (1 - alpha) * accel_roll;
      
      // Simulate heading (yaw) - you can add magnetometer for real heading
      yaw += gz * 0.1;
      if (yaw > 360) yaw -= 360;
      if (yaw < 0) yaw += 360;
      
      // Send orientation data in clean format
      Serial.print("Orientation: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(roll, 2);
      Serial.print(", ");
      Serial.println(pitch, 2);
      
      // Send additional data for completeness
      Serial.print("Accel: ");
      Serial.print(ax, 2);
      Serial.print(", ");
      Serial.print(ay, 2);
      Serial.print(", ");
      Serial.println(az, 2);
      
      Serial.print("Gyro: ");
      Serial.print(gx, 2);
      Serial.print(", ");
      Serial.print(gy, 2);
      Serial.print(", ");
      Serial.println(gz, 2);
    }
  }
} 