/*
 * Burst IMU Test for Arduino Nano RP2040 Connect
 * 
 * Collects IMU data in short bursts to prevent overheating
 * Works around thermal stability issues
 */

#include <Arduino_LSM6DSOX.h>
#include <math.h>

// Timing variables
unsigned long lastBurst = 0;
const unsigned long burstInterval = 5000; // 5 seconds between bursts
const unsigned long burstDuration = 2000; // 2 seconds of data collection
bool inBurst = false;
unsigned long burstStart = 0;

// Data collection
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("Burst IMU Test v1.0");
  Serial.println("Collecting data in 2-second bursts every 5 seconds");
  Serial.println("This prevents overheating issues");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check if we should start a burst
  if (!inBurst && (currentTime - lastBurst >= burstInterval)) {
    inBurst = true;
    burstStart = currentTime;
    lastBurst = currentTime;
    Serial.println("--- Starting Data Burst ---");
  }
  
  // Check if we should end the burst
  if (inBurst && (currentTime - burstStart >= burstDuration)) {
    inBurst = false;
    Serial.println("--- Ending Data Burst ---");
    Serial.println("Cooling down for 3 seconds...");
    delay(3000); // Cooling period
  }
  
  // Collect data during burst
  if (inBurst) {
    float ax, ay, az;
    float gx, gy, gz;
    
    if (IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      // Calculate orientation
      float accel_pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
      float accel_roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
      
      // Simple filter
      pitch = 0.95 * accel_pitch + 0.05 * (pitch + gy * 0.1);
      roll = 0.95 * accel_roll + 0.05 * (roll + gx * 0.1);
      yaw += gz * 0.1;
      
      // Send data
      Serial.print("Orientation: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(roll, 2);
      Serial.print(", ");
      Serial.println(pitch, 2);
      
      Serial.print("Accel: ");
      Serial.print(ax, 2);
      Serial.print(", ");
      Serial.print(ay, 2);
      Serial.print(", ");
      Serial.println(az, 2);
      
      delay(100); // 10Hz during burst
    }
  } else {
    // Idle period - minimal processing
    delay(100);
  }
} 