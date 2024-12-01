/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;  // variable to store the servo position
int angleMin = 45;          
int angleMax = 135;

void setup() {
  myservo.attach(11);     // attaches the servo on pin 9 to the Servo object
  Serial.begin(115200);  // initialize serial communications at 9600 bps:

}

void loop() {
  for (pos = angleMin; pos <= angleMax; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    Serial.println(myservo.read());
    delay(5);  // waits 15 ms for the servo to reach the position
  }
  for (pos = angleMax; pos >= angleMin; pos -= 1) {  // goes from 180 degrees to 0 degrees
    myservo.write(pos);                 // tell servo to go to position in variable 'pos'
    Serial.println(myservo.read());
    delay(5);  // waits 15 ms for the servo to reach the position
  }

  // myservo.write(45);  // tell servo to go to position in variable 'pos'
  // Serial.println(45);
  // delay(1000);
}
