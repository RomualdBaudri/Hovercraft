#include <Servo.h>

// Define signal ranges and pins
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 3
#define REVERSE_PIN 7  // Reverse signal wire connected to D7

Servo motor;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Program begin...");
  delay(1000);
  Serial.println("This program will start the ESC.");

  motor.attach(MOTOR_PIN);

  // Configure reverse pin
  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(REVERSE_PIN, LOW); // Start in forward mode by default

  // ESC calibration
  Serial.print("Now writing maximum output: (");
  Serial.print(MAX_SIGNAL);
  Serial.println(" us in this case)");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor.writeMicroseconds(MAX_SIGNAL);
  Serial.println("Step 1");

//   // Wait for input to select the menu
// while (!Serial.available());
//   Serial.println("Step 2");
//   Serial.read();
//   motor.writeMicroseconds(MIN_SIGNAL);
//   Serial.println("enter in the menu. Wait for selection of option");

//   // Wait for input to select the option
// while (!Serial.available());
//     Serial.println("Step 3");
//     Serial.read();
//     motor.writeMicroseconds(MAX_SIGNAL);
//     // Wait for input to be back in the programm selection menu and then go back to calibration 

while (!Serial.available());
  Serial.println("Step 4");

  Serial.read();
  // Send minimum output
  Serial.println("\n");
  Serial.print("Sending minimum output: (");
  Serial.print(MIN_SIGNAL);
  Serial.println(" us in this case)");
  motor.writeMicroseconds(MIN_SIGNAL);
  Serial.println("The ESC is calibrated.");
  Serial.println("----");
  Serial.println("Type a value between 1000 and 2000 to control the motor.");
  Serial.println("Send '1000' to stop the motor, '2000' for full throttle.");
  Serial.println("Send 'R' to toggle reverse mode.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input as a string

    // Handle reverse mode toggle
    if (input.equalsIgnoreCase("R")) {
      static bool isReverse = false; // Keep track of reverse state
      isReverse = !isReverse; // Toggle reverse state
      digitalWrite(REVERSE_PIN, isReverse ? HIGH : LOW); // Set reverse signal
      Serial.print("Reverse mode ");
      Serial.println(isReverse ? "activated." : "deactivated.");
    } 
    else { // Handle throttle control
      int throttle = input.toInt();
      if (throttle >= MIN_SIGNAL && throttle <= MAX_SIGNAL) {
        motor.writeMicroseconds(throttle); // Send throttle value to ESC

        // Calculate motor speed percentage
        float speedPercent = (throttle - MIN_SIGNAL) * 100.0 / (MAX_SIGNAL - MIN_SIGNAL);
        Serial.print("Motor speed: ");
        Serial.print(speedPercent);
        Serial.println("%");
      } else {
        Serial.println("Invalid input! Enter a value between 1000 and 2000 or 'R' to toggle reverse.");
      }
    }
  }
}
