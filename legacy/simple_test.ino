/*
 * Simple Arduino Test - No IMU Required
 * 
 * This sketch just sends basic messages to verify Arduino is working
 * No IMU libraries needed
 */

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Simple Arduino Test v1.0");
  Serial.println("This sketch doesn't use IMU - just basic communication");
  Serial.println("Arduino is working if you see this message!");
}

void loop() {
  static unsigned long lastMessage = 0;
  unsigned long currentTime = millis();
  
  // Send a message every 500ms
  if (currentTime - lastMessage >= 500) {
    lastMessage = currentTime;
    Serial.print("Arduino alive at ");
    Serial.print(currentTime);
    Serial.println(" ms");
  }
} 