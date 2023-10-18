#ifndef MotorWithEncoder_H
#define MotorWithEncoder_H
#include <Encoder.h>
#include <inttypes.h>

#include "Arduino.h"

class MotorWithEncoder {
   public:
    enum DriverMode { PHASE_ENABLE, PWM };

    //! IF YOU WANT TO ADD A NEW CONTROLLER MODE, MAKE SURE TO ADD IT TO setControllerMode FUNCTION IN .CPP FILE AS WELL
    enum ControllerMode {
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        FORCE_CONTROL,
    };
    static int resolutionBit;
    static int maxPulse;
    static int motorCount;

    MotorWithEncoder(uint8_t motorPin1, uint8_t motorPin2, uint8_t encoderPin1, uint8_t encoderPin2);

    
    double err = 0;
    void update();
    bool goTo(double);
    bool go();
    void brake();
    void release();
    void home();
    void rotate(int, double);
    void forceRotate(int, double);

    void resetEncoderPos();
    void setEncoderPos(int32_t);
    void setLimitSwitchPin(int8_t);
    void setDesiredPosition(long);
    void setDesiredVelocity(long);
    void setDesiredForce(double);
    void setForce(double);
    void setDeadBand(double);
    void setDriverMode(DriverMode);
    void setControllerMode(ControllerMode);
    void setCoefficient(double);
    void setFDMElementCount(int);
    void setVelocityWindowLength(int);
    void setKp(double);
    void setKi(double);
    void setKd(double);

    // Do not use this function unless you know what you are doing
    void resetIntegralTerm();

    int32_t getPos();
    double getVel();
    int32_t getAcc();
    uint8_t getPhasePin();
    uint8_t getEnablePin();
    ControllerMode getControllerMode();
    DriverMode getDriverMode();
    double getCoefficient();
    double getKp();
    double getKi();
    double getKd();

    void setHardPosLimit(int32_t, int32_t);
    void setSoftPosLimit(int32_t, int32_t);
    int32_t getUpperSoftPosLimit();
    int32_t getLowerSoftPosLimit();
    int32_t getUpperHardPosLimit();
    int32_t getLowerHardPosLimit();
    void removeHardPosLimit();
    void removeSoftPosLimit();

#if defined(__arm__) && defined(CORE_TEENSY)
    void setResolutionBit(int);
    int getResolutionBit();
#endif

    bool hardStop = false;
    bool softStop = false;
    
   private:
    int _motorIndex = 0;

    int32_t _position = 0;  // Unit is the encoder count
    int32_t _positionDesired = 0;
    int32_t _positionPrevious = 0;
    int32_t _positionErrorPrevious = 0;
    volatile int32_t _positionOffset = 0;

    double _count_to_mm = 1;

    double _pulse = 0;
    double* _lastVelocities;
    int32_t* _lastPositions;
    double* _lastAccelerations;
    int _positionIndex = 0;
    long long* _deltaTs;
    int _positionWindowLength = 2;
    int _velocityIndex = 0;
    int _velocityWindowLength = 150;  //5// Empirically determined
    int _accelerationWindowLength = 5;
    int _accelerationIndex = 0;
    int64_t _velocity = 0;  // Unit is encoder count per second
    int32_t _velocityDesired = 0;
    int32_t _velocityPrevious = 0;
    int32_t _velocityErrorPrevious = 0;
    int32_t _PIDErrorPrevious = 0;

    int32_t _acceleration = 0;  // Unit is encoder count per second squared
    int32_t _lowerHardPosLimit = 0;
    int32_t _upperHardPosLimit = 0;
    int32_t _lowerSoftPosLimit = 0;
    int32_t _upperSoftPosLimit = 0;

    int _fdmElementCount = 2;
    const int _errorWindowLength = 20;
    double* _lastErrors;
    int errorIndex = 0;
    long long _updateTimer = 0;
    long _steadyStateError = 0;

    double _forceDesired = 0;
    double _force = 0;

    bool _isSteadyState = false;
    
    enum HomingState{IDLE, MOVING_TO_LIMIT_SWITCH, MOVING_TO_ZERO};
    HomingState _homingState = HomingState::IDLE;
    bool _homing = false;
    bool _hasLimitSwitch = false;
    uint8_t _limitSwitchPin;
    bool _limitSwitchState = false;
    bool _limitSwitchStatePrev = false;
    bool _limitSwitchReading = false;
    bool _limitSwitchReadingPrev = false;
    unsigned long _limitSwitchDebounceTime = 0;

    uint8_t _phasePin;   // Phase pin for PHASE/ENABLE mode
    uint8_t _enablePin;  // Enable pin for PHASE/ENABLE mode
    Encoder* _encoder;   // Pointer to the encoder object, initialized when constructed.
    float _maxOutputPID =
        100000;  // When there are no Kd and Ki gains, decreasing this has the same effect as increasing Kp.

    DriverMode _driverMode =
        DriverMode::PWM;  // By default, the driver is controlled using PWM signals on both PWM pins.
    ControllerMode _controllerMode = ControllerMode::POSITION_CONTROL;
    double _KpPos = 1, _KiPos = 0,
           _KdPos = 0;  // By default, controller is only P-controller. Gains for the position controller
    double _KpVel = 1, _KiVel = 0,
           _KdVel = 0;  // By default, controller is only P-controller. Gains for the velocity controller
    double _KpFor = 0, _KiFor = 0,
           _KdFor = 0;     // By default, controller is only P-controller. Gains for the force controller
    double _deadBand = 0;  // Motor does not start rotating until a certain voltage is given. Set this value such that
                           // motor is almost rotating.

    long long _iTerm = 0;
    bool _hasHardPosLimit = false;
    bool _hasSoftPosLimit = false;
    bool _hardLimitViolation = false;
    bool _softLimitViolation = false;
    bool _isDirectionSafeToMove = true;
    bool _canMove = true;
    
    void _home();
    long long _calculatePositionPID(long);
    long long _calculateVelocityPID(long);
    double _calculatePID(double);
};
#endif
