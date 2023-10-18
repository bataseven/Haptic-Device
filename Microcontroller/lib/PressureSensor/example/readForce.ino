// Read force data using Pressure Sensor library
#include <PressureSensor.h>

// Create an instance of the PressureSensor class
PressureSensor sensor(A0);

long lastTime = 0;

void setup() {
    Serial.begin(9600);

    // Calibrate the sensor. This function is blocking(~200ms) but only called once.
    // Don't touch the sensor during calibration.
    sensor.calibrate();
}

void loop() {
    // Update the sensor
    sensor.update();

    // Calculate the force and print it every 100ms
    if (millis() - lastTime > 100) {
        lastTime = millis();
        double force = sensor.getForce();
        Serial.print("Force: ");
        Serial.print(force);
        Serial.println("N");
    }
}