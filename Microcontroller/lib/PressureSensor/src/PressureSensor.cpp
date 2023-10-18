#include "PressureSensor.h"

#include "Arduino.h"

int PressureSensor::resolutionBit = 12;
int PressureSensor::maxInput = pow(2, PressureSensor::resolutionBit);

// Constructor
PressureSensor::PressureSensor(uint8_t pinToRead) : _pinToRead(pinToRead) {
    _lastReadings = new int[_windowSize];
    memset(_lastReadings, 0, _windowSize * sizeof(*_lastReadings));
#if defined(__arm__) && defined(CORE_TEENSY)
    analogReadResolution(PressureSensor::resolutionBit);
    pinMode(_pinToRead, INPUT_PULLDOWN);
#else
    PressureSensor::resolutionBit = 10;
    PressureSensor::maxInput = pow(2, PressureSensor::resolutionBit);
    pinMode(_pinToRead, INPUT);
#endif
};

// This functions needs to be called whenever possible. It updates the latest readings and slides the window.
// i.e. in the loop() function.
void PressureSensor::update() {
    _reading = analogRead(_pinToRead);
    _lastReadings[_readingIndex] = _reading >= _zeroLevelNoise ? _reading : 0;
    _readingIndex = _readingIndex + 1 == _windowSize ? 0 : _readingIndex + 1;
}

// Returns a force estimate in Newtons from readings.
// If no argument is passed, force is calculated from the average of the latest readings.
double PressureSensor::getForce(bool calculateAverage = true) {
    float reading = calculateAverage ? this->getAverageReading() : this->getReading();
    if (reading < 0.5) return 0;                     // Prevent divide by zero
    float Vout = _Vs * reading / maxInput;           // Output voltage of the voltage divider
    _R1 = (_Vs * _R2) / Vout - _R2;                  // Resistance of the sensor
    double grams = _a * pow(_R1 / 1000.0, _b) + _c;  // Calculate force in grams, Curve is fitted in kOhms and grams
    double force = grams * 0.009816;                 // Convert to Newtons

    return _calibrated ? force - _calibrationOffset : _calibrate(force);
}

// Returns the latest read value from the analog pin
float PressureSensor::getReading() { return _reading; }

// Returns the average over the latest N values. N is _windowSize
float PressureSensor::getAverageReading() {
    int sum = 0;
    for (int i = 0; i < _windowSize; i++) sum += _lastReadings[i];
    return (float)sum / (float)_windowSize;
}

// Set the window size used in the moving average filter
void PressureSensor::setAveragingWindow(int windowSize) {
    _windowSize = windowSize;
    delete[] _lastReadings;
    _lastReadings = new int[_windowSize];
    memset(_lastReadings, 0, _windowSize * sizeof(*_lastReadings));
    _readingIndex = 0;
}

// Set the noise level that is considered zero
void PressureSensor::setZeroLevelNoise(int zeroLevelNoise) {
    zeroLevelNoise = constrain(zeroLevelNoise, 0, PressureSensor::maxInput);
    _zeroLevelNoise = zeroLevelNoise;
}

// Set the voltage of the power supply
void PressureSensor::setVs(float Vs) { _Vs = Vs; }

// Set the resistance of the voltage divider
void PressureSensor::setR2(float R2) { _R2 = R2; }

// Calibrates the sensor by taking the average of the first _maxNumberOfCalibrations readings.
// This function does not necessarily need to be called.
// If it is not called, the sensor will be calibrated after the first _maxNumberOfCalibrations getForce() calls.
// It only calibrates the force offset.
void PressureSensor::calibrate() {
    if (_calibrated) return;  // If already calibrated, do nothing
    while (!_calibrated) {
        this->update();
        this->getForce(true);
        delay(1);
    }
}

// Returns true if the sensor is calibrated
bool PressureSensor::isCalibrated() { return _calibrated; }

// First time getForce() is called, it will return 0. This function is called to calibrate the sensor.
int PressureSensor::_calibrate(double force) {
    if (_calibrationCount < _maxNumberOfCalibrations) {
        _calibrationSum += force;
        _calibrationCount++;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    } else {
        _calibrationOffset = _calibrationSum / _maxNumberOfCalibrations;
        _calibrated = true;
        digitalWrite(LED_BUILTIN, LOW);
    }
    return 0;
}