#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <Servo.h> // Servo library

// Define constants and pins
#define ANGLE_STEP 10         // Maximum angle step per update
#define DELAY_MS 25           // Delay between updates (in milliseconds)
#define SERVO_PIN 6           // Servo motor controlled by JSL
#define MOTOR_PIN_1 3         // ESC 1 pin
#define MOTOR_PIN_2 8         // ESC 2 pin (changed to a PWM-capable pin)
#define MAX_SIGNAL 1200       // Safe max signal for ESC
#define MIN_SIGNAL 1000       // Minimum signal for ESC
#define MOTOR_POWER_15 1150   // Motor power at 15% (constant)

// BLE service and characteristic UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RED_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define GREEN_UUID "beb5484e-36e1-4688-b7f5-ea07361b26a8"
#define BLUE_UUID "beb5485e-36e1-4688-b7f5-ea07361b26a8"
#define JSL_UUID "dbaf26fc-8ed7-47f6-9003-f72867b4de3c"
#define JSR_UUID "dbaf26fc-4ed7-47f6-9003-f72867b4de3c"
#define GONFLE_UUID "daaea4641-7cc2-4a4d-b34e-444ce4fea788"

// BLE service and characteristics
BLEService hovercraftService(SERVICE_UUID);
BLEByteCharacteristic redCharacteristic(RED_UUID, BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic greenCharacteristic(GREEN_UUID, BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic blueCharacteristic(BLUE_UUID, BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic JSL(JSL_UUID, BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic JSR(JSR_UUID, BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic gonfleCharacteristic(GONFLE_UUID, BLERead | BLEWrite | BLENotify);

// RGB LED pins (active low)
const int redPin = LEDR;
const int greenPin = LEDG;
const int bluePin = LEDB;

// Servo motors
Servo escMotor1;     // ESC for motor 1 (JSR)
Servo escMotor2;     // ESC for motor 2 (inflation control)
Servo servoMotor;    // Servo controlled by JSL

// BLE connection state
bool BLE_Connected = false;
int currentServoAngle = 90;    // Initial servo position
int currentThrottle = MIN_SIGNAL; // Initial throttle value (JSR)

// Function Prototypes
void setupBLE();
void handleBLE();
void updateLED(int pin, byte value);
void updateServo(int targetAngle);
void updateThrottle(int8_t throttleInput);
void handleGonfle(byte gonfleValue);
void initializeESC(Servo &motor, int motorNumber);

void setup() {
  Serial.begin(115200);

  // Initialize servo motors
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(currentServoAngle); // Set initial servo position
  escMotor1.attach(MOTOR_PIN_1);
  escMotor2.attach(MOTOR_PIN_2);

  // Calibrate ESCs
  initializeESC(escMotor1, 1);
  initializeESC(escMotor2, 2);

  // Initialize BLE
  setupBLE();

  // Set RGB LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  Serial.println("Setup complete!");
}

void loop() {
  handleBLE(); // Process BLE connections and characteristic writes
}

// Function Definitions

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setLocalName("Nano_BLE");
  BLE.setAdvertisedService(hovercraftService);

  // Add characteristics to the service
  hovercraftService.addCharacteristic(redCharacteristic);
  hovercraftService.addCharacteristic(greenCharacteristic);
  hovercraftService.addCharacteristic(blueCharacteristic);
  hovercraftService.addCharacteristic(JSL);
  hovercraftService.addCharacteristic(JSR);
  hovercraftService.addCharacteristic(gonfleCharacteristic);

  BLE.addService(hovercraftService);
  BLE.advertise();

  Serial.println("BLE device is now advertising...");
}

void handleBLE() {
  
  BLE.poll();

  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    if (!BLE_Connected) {
      BLE_Connected = true;
      Serial.println("Device connected!");
    }

    // Handle characteristic writes
    if (redCharacteristic.written()) updateLED(redPin, redCharacteristic.value());
    if (greenCharacteristic.written()) updateLED(greenPin, greenCharacteristic.value());
    if (blueCharacteristic.written()) updateLED(bluePin, blueCharacteristic.value());

    if (JSL.written()) updateServo(JSL.value());
    if (JSR.written()) updateThrottle(JSR.value());
    if (gonfleCharacteristic.written()) handleGonfle(gonfleCharacteristic.value());
  } else if (BLE_Connected) {
    BLE_Connected = false;
    Serial.println("Device disconnected!");
  }
}

void updateLED(int pin, byte value) {
  digitalWrite(pin, value == 1 ? LOW : HIGH); // Active low
  Serial.print("LED on pin ");
  Serial.print(pin);
  Serial.print(" set to ");
  Serial.println(value == 1 ? "ON" : "OFF");
}

void updateServo(int targetAngle) {
  static unsigned long lastUpdateTime = 0;
  static int currentServoAngle = 0; // Assurez-vous que cette variable est déclarée quelque part globalement ou statiquement.

  Serial.print("Joystick byte received: ");
  Serial.print(targetAngle);
  Serial.print(", Target servo angle: ");
  Serial.println(targetAngle);

  // Vérifiez si un délai s'est écoulé depuis la dernière mise à jour
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= DELAY_MS) {
    lastUpdateTime = currentTime;

    // Calculez le prochain angle en fonction de la direction
    if (currentServoAngle < targetAngle) {
      currentServoAngle = min(currentServoAngle + ANGLE_STEP, targetAngle);
    } else if (currentServoAngle > targetAngle) {
      currentServoAngle = max(currentServoAngle - ANGLE_STEP, targetAngle);
    }

    // Mettez à jour la position du servomoteur uniquement si nécessaire
    servoMotor.write(currentServoAngle);
    Serial.print("Current Servo Angle: ");
    Serial.println(currentServoAngle);
  }
}


void updateThrottle(int8_t throttleInput) {
  throttleInput = constrain(throttleInput, 0, 100); // Ensure within range
  int targetThrottle = map(throttleInput, 0, 100, MIN_SIGNAL, MAX_SIGNAL);
  escMotor1.writeMicroseconds(targetThrottle);
  currentThrottle = targetThrottle;

  Serial.print("Throttle set to: ");
  Serial.println(targetThrottle);
}

void handleGonfle(byte gonfleValue) {
  if (gonfleValue == 1) {
    escMotor2.writeMicroseconds(MOTOR_POWER_15);
    Serial.println("Motor set to 15% power");
  } else {
    escMotor2.writeMicroseconds(MIN_SIGNAL);
    Serial.println("Motor set to 0% power");
  }
}

void initializeESC(Servo &motor, int motorNumber) {
  Serial.print("Setting ESC to LOW "); Serial.println(motorNumber);

  motor.writeMicroseconds(MIN_SIGNAL);

  Serial.print("ESC "); Serial.print(motorNumber); Serial.println(" Setting complete.");
}
