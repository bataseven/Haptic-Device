/*
  PressureSensor.h - Library for reading the force applied to a pressure sensor(FSR).
  Created by Berke Ataseven, 2021.

 Schematic needs to be as follows:

             Vout 
       R1     |      R2
 -----====---------=====----
 |                         |
 |                         |
 ---------- (+ -) ----------
           Vs   GND

 Vout = Vs * R2 / (R1 + R2)

 R1: Resistance of the sensor(FSR) in Ohms
 R2: Resistance of the voltage divider in Ohms
 Vs: Voltage of the power supply in Volts

 Force is calculated using the following formula:
 F = a * (R1 ^ b) + c

*/
#ifndef PressureSensor_H
#define PressureSensor_H
#include "Arduino.h"

class PressureSensor {
   public:
    PressureSensor(uint8_t);
    void update();
    double getForce(bool = true);
    float getReading();
    float getAverageReading();
    void setAveragingWindow(int);
    void setZeroLevelNoise(int);
    void setVs(float);
    void setR2(float);
    void calibrate();
    bool isCalibrated();

    static int resolutionBit;
    static int maxInput;

   private:
    int _calibrate(
        double);  // First time getForce() is called, it will return 0. This function is called to calibrate the sensor.
    bool _calibrated = false;            // True if the sensor has been calibrated
    int _calibrationCount = 0;           // Keep track of the calibration readings
    int _maxNumberOfCalibrations = 200;  // Number of calibration readings
    double _calibrationSum = 0;          // Sum of the calibration readings
    double _calibrationOffset = 0;       // Offset used to calibrate the sensor
    uint8_t _pinToRead;                  // Pin to read
    int _zeroLevelNoise = 5;             // Noise level that is considered zero
    int _windowSize = 20;                // Window size used in the moving average filter
    int _readingIndex = 0;               // Index of the latest reading
    int _reading = 0;                    // Latest reading
    int _readingAveraged = 0;            // Average of the latest readings
    int* _lastReadings;                  // Array of the latest readings
    float _R1 = 0;                       // Resistance of the sensor in Ohms
    float _R2 = 2000;                    // Resistance of the voltage divider in Ohms
    float _Vs = 3.3;                     // Voltage of the power supply in Volts
    float _a = 1069;                     // Coefficient a of the force formula
    float _b = -1.620;                   // Coefficient b of the force formula
    float _c = 101;                      // Coefficient c of the force formula
};
#endif