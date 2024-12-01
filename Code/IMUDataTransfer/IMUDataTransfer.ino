#include <Arduino_BMI270_BMM150.h>

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    Serial.println("IMU initialized.");
}

void loop() {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
        // Read sensor values
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        IMU.readMagneticField(mx, my, mz);

        // Print data in labeled format
        Serial.print("ax:"); Serial.print(ax, 6); Serial.print("\t");
        Serial.print("ay:"); Serial.print(ay, 6); Serial.print("\t");
        Serial.print("az:"); Serial.print(az, 6); Serial.print("\t");

        Serial.print("gx:"); Serial.print(gx, 6); Serial.print("\t");
        Serial.print("gy:"); Serial.print(gy, 6); Serial.print("\t");
        Serial.print("gz:"); Serial.print(gz, 6); Serial.print("\t");

        Serial.print("mx:"); Serial.print(mx, 6); Serial.print("\t");
        Serial.print("my:"); Serial.print(my, 6); Serial.print("\t");
        Serial.print("mz:"); Serial.print(mz, 6); Serial.print("\n");
    }

    delay(10);
}
