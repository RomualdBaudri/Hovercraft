#include <Servo.h>

Servo myServo; 
int pos = 0;
int angleMin = 50;
int angleMax = 130;

// MAXIMUM ANGLE STEP FOR 25 DELAY IS 10° IN THE FOR LOOP (pos+=10) IN 1ms IT DOES 0.429° AT THE MAXIMUM

void setup() {
  myServo.attach(9);
  Serial.begin(115200);
  myServo.write((angleMin + angleMax) / 2); // Move to a neutral position
  delay(1000); // Let the servo stabilize
}

void loop() {
  static int currentPos = (angleMin + angleMax) / 2; // Start at neutral position
  
  // Sweep from angleMin to angleMax
  for (pos = angleMin; pos <= angleMax; pos += 10) {
    currentPos = currentPos + (pos - currentPos) / 2; // Smooth transition
    myServo.write(currentPos); // Write the smoothed position
    delay(25); // Stabilization delay
  }

  // Sweep back from angleMax to angleMin
  for (pos = angleMax; pos >= angleMin; pos -= 10) {
    currentPos = currentPos + (pos - currentPos) / 2; // Smooth transition
    myServo.write(currentPos); // Write the smoothed position
    delay(25); // Stabilization delay
  }
}
