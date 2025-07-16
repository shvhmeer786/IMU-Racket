/*
 * Stable Real IMU for Arduino Nano RP2040 Connect
 * 
 * This sketch uses the built-in IMU with a more compatible approach
 * Works without needing additional library installations
 * 
 * Upload this to your Arduino using Arduino IDE
 */

#include <Arduino_LSM6DSOX.h>
#include <math.h>

// Timing for consistent data rate
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100; // 100ms = 10Hz

// Simple orientation tracking
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Don't wait for Serial Monitor - start immediately
  delay(1000);
  
  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    Serial.println("Please check connections and library installation");
    while (1) {
      delay(1000);
    }
  }
  
  // Startup messages
  Serial.println("Stable Real IMU v1.0");
  Serial.println("Sending real IMU orientation data...");
  Serial.println("Move the Arduino to see changes!");
  
  delay(500);
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
      
      // Simple complementary filter (95% accelerometer, 5% gyro)
      pitch = 0.95 * accel_pitch + 0.05 * (pitch + gy * 0.1);
      roll = 0.95 * accel_roll + 0.05 * (roll + gx * 0.1);
      
      // Integrate gyroscope for yaw (heading)
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
      
      // Calculate and send linear acceleration (without gravity)
      float lin_ax = ax - sin(roll * PI / 180.0) * sin(pitch * PI / 180.0);
      float lin_ay = ay - cos(roll * PI / 180.0) * sin(pitch * PI / 180.0);
      float lin_az = az - cos(pitch * PI / 180.0) * cos(roll * PI / 180.0);
      
      Serial.print("LinAccel: ");
      Serial.print(lin_ax, 2);
      Serial.print(", ");
      Serial.print(lin_ay, 2);
      Serial.print(", ");
      Serial.println(lin_az, 2);
      
      // Ensure data is sent immediately
      Serial.flush();
    } else {
      // If IMU read fails, send error message
      Serial.println("IMU read failed");
    }
  }
  
  // Small delay to prevent overwhelming the serial buffer
  delay(10);
} 