#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <Servo.h> // Servo library

// Define constants and pins
#define ANGLE_STEP 10 // Maximum angle step per update
#define DELAY_MS 25   // Delay between updates (in milliseconds)
#define SERVO_PIN 9   // Servo motor controlled by JSL
#define MOTOR_PIN 3 
#define REVERSE_PIN 7 // Reverse signal wire connected to D7
#define MAX_SIGNAL 1200 // Safe max signal for ESC (JSR) Otherwise the motor is at risk of burning.
#define MIN_SIGNAL 1000 // Minimum signal for ESC (JSR)

// BLE service and characteristics
BLEService LEDService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
BLEByteCharacteristic redCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic greenCharacteristic("beb5484e-36e1-4688-b7f5-ea07361b26a8", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic blueCharacteristic("beb5485e-36e1-4688-b7f5-ea07361b26a8", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic JSL("dbaf26fc-8ed7-47f6-9003-f72867b4de3c", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic JSR("dbaf26fc-4ed7-47f6-9003-f72867b4de3c", BLERead | BLEWrite | BLENotify);

// RGB LED pins
const int redPin = LEDR;
const int greenPin = LEDG;
const int bluePin = LEDB;

// Servo motor pin
Servo escMotor; // ESC controlled by JSR
Servo servoMotor; // Servo controlled by JSL

// BLE Connection State
bool BLE_Connected = false;
int currentServoAngle = 90; // Initial servo position
int currentThrottle = MIN_SIGNAL; // Initial throttle value (JSR)

void setup() {
  Serial.begin(115200);

  // Initialize Servo
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(currentServoAngle); // Set initial servo position
  delay(1000); // Stabilize servo

  escMotor.attach(MOTOR_PIN);
  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(REVERSE_PIN, LOW); // Start in forward mode by default
  delay(1000); // Stabilize motor

  Serial.println("Calibrating ESC...");
  escMotor.writeMicroseconds(MAX_SIGNAL);
  delay(2100);
  escMotor.writeMicroseconds(MIN_SIGNAL);
  delay(2100);
  Serial.println("ESC calibration complete.");

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setLocalName("Nano_BLE");
  BLE.setAdvertisedService(LEDService);
  BLE.setConnectionInterval(7, 7);

  // Add characteristics to the service
  LEDService.addCharacteristic(redCharacteristic);
  LEDService.addCharacteristic(greenCharacteristic);
  LEDService.addCharacteristic(blueCharacteristic);
  LEDService.addCharacteristic(JSL);
  LEDService.addCharacteristic(JSR);
  BLE.addService(LEDService);
  BLE.advertise();
  Serial.println("BLE device is now advertising...");
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  BLE.poll();

  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    if (!BLE_Connected) {
      BLE_Connected = true;
      Serial.println("Device connected!");
    }

    // Handle RGB characteristic writes
    if (redCharacteristic.written()) {
      byte redValue = redCharacteristic.value();
      digitalWrite(redPin, redValue == 1 ? HIGH : LOW);
      Serial.print("Red LED pin set to: ");
      Serial.println(redValue);
    }
    if (greenCharacteristic.written()) {
      byte greenValue = greenCharacteristic.value();
      digitalWrite(greenPin, greenValue == 1 ? HIGH : LOW);
      Serial.print("Green LED pin set to: ");
      Serial.println(greenValue);
    }
    if (blueCharacteristic.written()) {
      byte blueValue = blueCharacteristic.value();
      digitalWrite(bluePin, blueValue == 1 ? HIGH : LOW);
      Serial.print("Blue LED pin set to: ");
      Serial.println(blueValue);
    }
    // Handle joystick left (JSL) input
    if (JSL.written()) {
      int  joystickValue = JSL.value(); // Read joystick value
      int targetServoAngle = joystickValue;

      Serial.print("Joystick byte received: ");
      Serial.print(joystickValue);
      Serial.print(", Target servo angle: ");
      Serial.println(targetServoAngle);

      // Smoothly move to the target position
      while (currentServoAngle != targetServoAngle) {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= DELAY_MS) {
          lastUpdateTime = currentTime;

          // Calculate step direction and move incrementally
          if (currentServoAngle < targetServoAngle) {
            currentServoAngle = min(currentServoAngle + ANGLE_STEP, targetServoAngle);
          } else if (currentServoAngle > targetServoAngle) {
            currentServoAngle = max(currentServoAngle - ANGLE_STEP, targetServoAngle);
          }

          servoMotor.write(currentServoAngle);
          Serial.print("Current Servo Angle: ");
          Serial.println(currentServoAngle);
        }
      }
    }
    // Handle joystick right (JSR) input for ESC motor
    if (JSR.written()) {
      int8_t  throttleInput = JSR.value(); // Expected range: 0 to 100
      if (throttleInput < 0) throttleInput = 0; // Ensure throttleInput is not negative

      int targetThrottle = map(throttleInput, 0, 100, MIN_SIGNAL, MAX_SIGNAL); // Map to ESC range
      escMotor.writeMicroseconds(targetThrottle); // Set ESC throttle
      currentThrottle = targetThrottle; // Update current throttle

      // Debugging information
      Serial.print("JSR: Throttle input = ");
      Serial.print(throttleInput);
      Serial.print(", Mapped throttle = ");
      Serial.println(targetThrottle);
    }
  } else if (BLE_Connected) {
    BLE_Connected = false;
    Serial.println("Device disconnected!");
  }
}
