#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <Servo.h> // Servo library

#define ANGLE_STEP 10 // Maximum angle step per update
#define DELAY_MS 25   // Delay between updates (in milliseconds)

// BLE service and characteristics
BLEService LEDService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
BLEByteCharacteristic redCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic greenCharacteristic("beb5484e-36e1-4688-b7f5-ea07361b26a8", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic blueCharacteristic("beb5485e-36e1-4688-b7f5-ea07361b26a8", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic ballCharacteristic("dbaf26fc-8ed7-47f6-9003-f72867b4de3c", BLERead | BLEWrite | BLENotify);

// RGB LED pins
const int redPin = LEDR;
const int greenPin = LEDG;
const int bluePin = LEDB;

// Servo motor pin
const int servoPin = 9;
Servo servoMotor;

// BLE Connection State
bool BLE_Connected = false;
int currentServoAngle = 90; // Initial servo position

void setup() {
  Serial.begin(115200);

  // Initialize Servo
  servoMotor.attach(servoPin);
  servoMotor.write(currentServoAngle); // Set initial servo position
  delay(1000); // Stabilize servo

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
  LEDService.addCharacteristic(ballCharacteristic);

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
    // Handle ball characteristic writes
    if (ballCharacteristic.written()) {
      uint8_t joystickValue = ballCharacteristic.value(); // Read joystick value
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
  } else if (BLE_Connected) {
    BLE_Connected = false;
    Serial.println("Device disconnected!");
  }
}
