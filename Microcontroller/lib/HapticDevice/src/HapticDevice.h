#define RAD2DEG(x) ((x)*180.0 / M_PI)
#define PRINT_DEBUG  // Prints debug messages if defined.

#ifndef HapticDevice_H
#define HapticDevice_H
#include <EEPROM.h>

#include "AdmittanceController.h"
#include "Arduino.h"
// #include "BNO055_support.h"
#include "DFRobot_BMX160.h"
#include "FunctionalButton.h"
#include "MotorWithEncoder.h"
#include "OctoWS2811.h"
#include "PressureSensor.h"
#include "WaveformGenerator.h"

#define PWM_FREQUENCY 18000
#define CONTROL_LOOP_FREQUENCY 1000

#define FINGER_LOWER_SOFT_LIMIT 0       // 0
#define FINGER_UPPER_SOFT_LIMIT 160000  // 160000
#define RIGID_BODY_STIFFNESS_THRESHOLD \
    12                              // Virtual objects with stiffness more than this value are considered rigid bodies.
#define PALM_LOWER_SOFT_LIMIT 550   // 500
#define PALM_UPPER_SOFT_LIMIT 7200  // 6800 // 7400 // 7350
#define PALM_LOWER_HARD_LIMIT 400
#define PALM_UPPER_HARD_LIMIT 7500

#define EEPROM_ADDRESS_PID_GAINS 0

#define RED 0x160000
#define GREEN 0x001600
#define PURPLE 0x160016
#define CYAN 0x001616
#define BLUE 0x000016
#define YELLOW 0x141200
#define PINK 0x120009
#define ORANGE 0x130500
#define WHITE 0x101010
#define BLACK 0x000000

const double COUNT_TO_MM = 29.5 / 99510.0;  // 26.1 / 107493.0
const double COUNT_TO_MM_2 = (6.8 * PI) / (12 * 298);

struct PseudoObject {
    float nominalWidth = 0;
    float stiffness = 0;
    float damping = 0;
    bool inContact = false;
    bool inContactPrev = false;
};

struct WSG50 {
    double width;
    double speed;
    double force;
};

class HapticDevice {
   private:
    Stream* serial;

    int fingerButtonState;
    int palmButtonState;

    unsigned long _encoderResetLEDTimer;

    /* OCTO LED*/
    static const int _numPins = 1;
    byte _pinList[_numPins] = {5};
    static const int _ledsPerStrip = 2;
    int _currentColor = 0;
    int _lastColor = 0;

    // These buffers need to be large enough for all the pixels.
    // The total number of pixels is "ledsPerStrip * numPins".
    // Each pixel needs 3 bytes, so multiply by 3.  An "int" is
    // 4 bytes, so divide by 4.  The array is created using "int"
    // so the compiler will align it to 32 bit memory.
    static const int _bytesPerLED = 3;  // change to 4 if using RGBW
    int _displayMemory[_ledsPerStrip * _numPins * _bytesPerLED / 4];
    int _drawingMemory[_ledsPerStrip * _numPins * _bytesPerLED / 4];

    const int _LEDConfig = WS2811_RGB | WS2811_800kHz;
    /* END OCTO LED*/

    // struct bno055_t _BNO055;
    unsigned long _imuUpdateTimer;
    const float _imuUpdateFrequency = 100;  // Hz
    long _imuLEDTimer = -2000;
    bool _imuActivationChanged = false;

    uint8_t fingerResetPin = 10;
    uint8_t palmResetPin = 12;

    void _LEDUpdate();

   public:
    HapticDevice();

    static MotorWithEncoder* finger;
    static MotorWithEncoder* palm;

    PressureSensor* thumbInnerSensor;
    PressureSensor* thumbOuterSensor;
    PressureSensor* indexInnerSensor;
    PressureSensor* indexOuterSensor;

    FunctionalButton* fingerButton;
    FunctionalButton* palmButton;

    OctoWS2811 led = OctoWS2811(_ledsPerStrip, _displayMemory, _drawingMemory, _LEDConfig, _numPins, _pinList);

    // struct bno055_euler eulerAngles;  // Structure to hold the Euler data
    bool imuActive = false;  // Initialize the imu as active

    WaveformGenerator waveformGeneratorFinger;
    WaveformGenerator waveformGeneratorPalm;
    float generatedFingerWave;
    float generatedPalmWave;

    AdmittanceController IOAC =
        AdmittanceController(2, 10, 1.0 / CONTROL_LOOP_FREQUENCY, 4);  // Mass, Damping, dt, Gain

    PseudoObject obj;
    WSG50 wsg50;

    double desiredFingerForce = 0;
    double netFingerForce = 0;
    double desiredFingerVel = 0;

    bool steadyState = false;

    void initialize();
    void update();
    void fingerControlLoop();
    void palmControlLoop();

    static void resetFingerEncoder();
    static void resetPalmEncoder();

    void setSerial(Stream& serial);
};
#endif