/*
 * Test Live IMU for Arduino Nano RP2040 Connect
 * 
 * This sketch sends realistic fake IMU data for immediate testing
 * Perfect for testing your live visualization system
 * 
 * Upload this to your Arduino using Arduino IDE
 */

#include <math.h>

// Timing for consistent data rate
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 100; // 100ms = 10Hz

// Smooth animation counter
float counter = 0;

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection (up to 3 seconds)
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime) < 3000) {
    delay(10);
  }
  
  // Startup messages
  Serial.println("Test Live IMU v1.0 - Fake Data");
  Serial.println("Sending realistic fake orientation data...");
  Serial.println("This will test your live visualization system!");
  
  delay(1000); // Brief pause for stability
}

void loop() {
  unsigned long currentTime = millis();
  
  // Send data at consistent intervals
  if (currentTime - lastUpdate >= updateInterval) {
    lastUpdate = currentTime;
    counter += 0.1;
    
    // Generate smooth, realistic fake orientation changes
    float heading = 180 + 120 * sin(counter * 0.05);  // Slow swing 60° each way
    float roll = 45 * sin(counter * 0.08);             // Medium roll motion
    float pitch = 30 * cos(counter * 0.06);            // Slow pitch motion
    
    // Keep heading in 0-360 range
    if (heading < 0) heading += 360;
    if (heading > 360) heading -= 360;
    
    // Send orientation data in clean format
    Serial.print("Orientation: ");
    Serial.print(heading, 2);
    Serial.print(", ");
    Serial.print(roll, 2);
    Serial.print(", ");
    Serial.println(pitch, 2);
    
    // Send realistic accelerometer data
    Serial.print("Accel: ");
    Serial.print(0.1 * sin(counter * 0.3), 2);
    Serial.print(", ");
    Serial.print(0.1 * cos(counter * 0.3), 2);
    Serial.print(", ");
    Serial.println(9.81 + 0.2 * sin(counter * 0.4), 2);
    
    // Send realistic gyroscope data
    Serial.print("Gyro: ");
    Serial.print(10 * sin(counter * 0.2), 2);
    Serial.print(", ");
    Serial.print(8 * cos(counter * 0.25), 2);
    Serial.print(", ");
    Serial.println(5 * sin(counter * 0.15), 2);
  }
} 