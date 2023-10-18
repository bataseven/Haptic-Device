/********************************************************************

  - MotorWithEncoder.cpp
  - A non-blocking PID control library for a DC motor with 2 encoder pins.
  - This library was tested with DRV8835 driver IC and Teensy 4.0.
  - 2 working modes of the driver IC are (PWM-PWM , PHASE-ENABLE) available
  - 2 control modes are (velocity & position control) available
  - Constrain the motor position with hard & soft limits (Coast and brake functionality of the
driver IC)

  - Created by Berke Ataseven, 2021.

*********************************************************************/

#include "MotorWithEncoder.h"

int MotorWithEncoder::resolutionBit = 8;  // This determines the analogWrite resolution bit. Is the
                                          // same for every instance of this class.
int MotorWithEncoder::maxPulse =
    pow(2, MotorWithEncoder::resolutionBit) - 1;  // Determines the maximum valid pulse for the analogWrite
                                                  // function. Is the same for every instance of this class.

int MotorWithEncoder::motorCount = 0;  // This is used to assign a unique index to each motor object. Is unique
                                       // for every instance of this class.

// Construct the object with the motor pins and the encoder pins.
MotorWithEncoder::MotorWithEncoder(uint8_t motorPin1, uint8_t motorPin2, uint8_t encoderPin1, uint8_t encoderPin2)
    : _phasePin(motorPin1), _enablePin(motorPin2) {
    _encoder = new Encoder(encoderPin1, encoderPin2);
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
#if defined(__arm__) && defined(CORE_TEENSY)
    analogWriteResolution(MotorWithEncoder::resolutionBit);
#endif
    _lastErrors = new double[_errorWindowLength];
    _lastVelocities = new double[_velocityWindowLength];
    _lastPositions = new int32_t[_positionWindowLength];
    _lastAccelerations = new double[_accelerationWindowLength];
    _deltaTs = new long long[_positionWindowLength];
    _motorIndex = motorCount++;
}

// Reads the encoder and updates the position, velocity, acceleration attributes.
// Also checks if the position limits are violated (If has a position limit)
// If a limit is violated or a stop flag is set, motor stops rotating.
// This should be called before anything else in each loop.
void MotorWithEncoder::update() {
    _position = _encoder->read() - _positionOffset;
    long long now = micros();
    _deltaTs[_positionIndex] = now - _updateTimer;
    _lastPositions[_positionIndex] = _position;

    // Calculate the velocity using backward finite difference
    // Coefficients are calculated using the link below
    // https://web.media.mit.edu/~crtaylor/calculator.html
    long sum = 0;
    for (int i = 0; i < _positionWindowLength; i++) sum += _deltaTs[i];
    double hAverage = (double)sum / (double)_positionWindowLength / 1e6;

    int64_t instantaneousVelocity = 0;

    switch (_fdmElementCount) {
        case 5:
            // (3*f[i-4]-16*f[i-3]+36*f[i-2]-48*f[i-1]+25*f[i+0])/(12*1.0*h**1)
            instantaneousVelocity =
                (3 * _lastPositions[(_positionIndex - 4 + _positionWindowLength) % _positionWindowLength] -
                 16 * _lastPositions[(_positionIndex - 3 + _positionWindowLength) % _positionWindowLength] +
                 36 * _lastPositions[(_positionIndex - 2 + _positionWindowLength) % _positionWindowLength] -
                 48 * _lastPositions[(_positionIndex - 1 + _positionWindowLength) % _positionWindowLength] +
                 25 * _lastPositions[(_positionIndex - 0 + _positionWindowLength) % _positionWindowLength]) /
                (12 * hAverage);
            break;

        case 3:
            // (1*f[i-2]-4*f[i-1]+3*f[i+0])/(2*1.0*h**1)
            instantaneousVelocity =
                (1 * _lastPositions[(_positionIndex - 2 + _positionWindowLength) % _positionWindowLength] -
                 4 * _lastPositions[(_positionIndex - 1 + _positionWindowLength) % _positionWindowLength] +
                 3 * _lastPositions[(_positionIndex - 0 + _positionWindowLength) % _positionWindowLength]) /
                (2 * hAverage);
            break;
        case 2:
        default:
            // -1*f[i-1]+1*f[i+0])/(1*1.0*h**1)
            instantaneousVelocity =
                (-1 * _lastPositions[(_positionIndex - 1 + _positionWindowLength) % _positionWindowLength] +
                 1 * _lastPositions[(_positionIndex - 0 + _positionWindowLength) % _positionWindowLength]) /
                hAverage;
            break;
    }

    _lastVelocities[_velocityIndex] = instantaneousVelocity;

    // Moving average filter for velocity
    int64_t velocitySum = 0;
    for (int i = 0; i < _velocityWindowLength; i++) velocitySum += _lastVelocities[i];
    _velocity = (double)velocitySum / (double)_velocityWindowLength;

    // Calculate the velocity using backward finite difference
    // Coefficients are calculated using the link below
    // https://web.media.mit.edu/~crtaylor/calculator.html
    int64_t instantaneousAcceleration =
        (11 * _lastPositions[(_positionIndex - 4 + _positionWindowLength) % _positionWindowLength] -
         56 * _lastPositions[(_positionIndex - 3 + _positionWindowLength) % _positionWindowLength] +
         114 * _lastPositions[(_positionIndex - 2 + _positionWindowLength) % _positionWindowLength] -
         104 * _lastPositions[(_positionIndex - 1 + _positionWindowLength) % _positionWindowLength] +
         35 * _lastPositions[(_positionIndex - 0 + _positionWindowLength) % _positionWindowLength]) /
        (12 * hAverage * hAverage);

    _lastAccelerations[_accelerationIndex] = instantaneousAcceleration;

    // Moving average filter for acceleration
    int64_t accelerationSum = 0;
    for (int i = 0; i < _accelerationWindowLength; i++) accelerationSum += _lastAccelerations[i];
    _acceleration = (double)accelerationSum / (double)_accelerationWindowLength;

    _positionPrevious = _position;
    _velocityPrevious = _velocity;
    _positionIndex = (_positionIndex + 1) % _positionWindowLength;
    _velocityIndex = (_velocityIndex + 1) % _velocityWindowLength;
    _accelerationIndex = (_accelerationIndex + 1) % _accelerationWindowLength;
    _updateTimer = now;

    _softLimitViolation =
        _hasSoftPosLimit && (_position >= _upperSoftPosLimit || _position <= _lowerSoftPosLimit) ? true : false;
    _hardLimitViolation =
        _hasHardPosLimit && (_position >= _upperHardPosLimit || _position <= _lowerHardPosLimit) ? true : false;

    if (_softLimitViolation || _hardLimitViolation) {
        bool upperLimitViolation = _position >= min(_upperSoftPosLimit, _upperHardPosLimit);
        if (upperLimitViolation && _pulse > 0)
            _isDirectionSafeToMove = false;
        else if (upperLimitViolation && _pulse <= 0)
            _isDirectionSafeToMove = true;
        else if (!upperLimitViolation && _pulse < 0)
            _isDirectionSafeToMove = false;
        else if (!upperLimitViolation && _pulse >= 0)
            _isDirectionSafeToMove = true;
    }

    if (hardStop || (_hardLimitViolation && !_isDirectionSafeToMove))
        this->brake();
    else if (softStop || (_softLimitViolation && !_isDirectionSafeToMove))
        this->release();

    if (_softLimitViolation || _hardLimitViolation)
        _canMove = _isDirectionSafeToMove && !hardStop && !softStop;
    else
        _canMove = !(hardStop || softStop);

    if (!_canMove) this->resetIntegralTerm();

    // Debounce the limit switch if it is present
    if (_hasLimitSwitch) {
        _limitSwitchReading = digitalRead(_limitSwitchPin);
        if (_limitSwitchReading != _limitSwitchReadingPrev) _limitSwitchDebounceTime = millis();
        if ((millis() - _limitSwitchDebounceTime) > 30) _limitSwitchState = _limitSwitchReading;
        _limitSwitchReadingPrev = _limitSwitchReading;
    }

    _home();
    _limitSwitchStatePrev = _limitSwitchState;
}

// Moves the motor as per the commanded value.
// Control loop frequency is determined by how frequent this function is called.
// Returns true if the target value is reached
bool MotorWithEncoder::goTo(double commandedValue) {
    // Override the commanded value if the motor is homing
    if (_homing) return false;

    double averageError = 0;
    bool isSteady = false;
    double valueToCalculateErrorFor = 0;

    _pulse = _calculatePID(commandedValue);

    if (!_canMove) {
        _isSteadyState = isSteady;
        return isSteady;
    }

    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL: {
            valueToCalculateErrorFor = _position;
        } break;

        case ControllerMode::VELOCITY_CONTROL: {
            valueToCalculateErrorFor = _velocity;
        } break;

        case ControllerMode::FORCE_CONTROL: {
            valueToCalculateErrorFor = _force;
        } break;
    }

    switch (_driverMode) {
        case DriverMode::PWM:
            if (_isSteadyState && _controllerMode == FORCE_CONTROL) break;
            if (_pulse > 5) {
                analogWrite(_phasePin, abs(_pulse));
                analogWrite(_enablePin, 0);
            } else if (_pulse < -5) {
                analogWrite(_phasePin, 0);
                analogWrite(_enablePin, abs(_pulse));
            } else {
                this->release();
            }
            break;
        case DriverMode::PHASE_ENABLE:  // TODO
            this->release();
            break;
    }

    if (commandedValue == 0)                                  // Avoid division by zero
        _lastErrors[errorIndex] = -valueToCalculateErrorFor;  // Calculate percent error
    else
        _lastErrors[errorIndex] = (commandedValue - valueToCalculateErrorFor) / (commandedValue);

    errorIndex = (errorIndex + 1) % _errorWindowLength;

    double sum = 0;
    for (int i = 0; i < _errorWindowLength; i++) sum += abs(_lastErrors[i]);

    averageError = sum / (double)_errorWindowLength;

    if (averageError <= 0.02) {  // Steady state is reached if the error is less than 2 percent
        _iTerm = 0;
        isSteady = true;
        _steadyStateError = averageError;
    }
    // if (isSteady && _controllerMode == VELOCITY_CONTROL) this->release();
    _isSteadyState = isSteady;
    return isSteady;
}

// Moves the motor as per the "desired value".
// Use setDesiredPosition() function to set the desired position.
// Use setDesiredVelocity() function to set the desired velocity.
// Use setDesiredForce() function to set the desired force.
// Returns true if the desired goal is reached.
bool MotorWithEncoder::go() {
    bool isSteady = false;
    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL:
            isSteady = this->goTo(_positionDesired);
            break;
        case ControllerMode::VELOCITY_CONTROL:
            isSteady = this->goTo(_velocityDesired);
            break;
        case ControllerMode::FORCE_CONTROL:
            isSteady = this->goTo(_forceDesired);
            break;
    }
    return isSteady;
}

// When the motor is moving, "brake()" and "release()"
// effectively stops the motor from rotating.
// However "brake()" does it immideatly, whereas, "release()" coasts.
void MotorWithEncoder::brake() {
    if (_driverMode == DriverMode::PWM) {
        analogWrite(_enablePin, maxPulse);
        analogWrite(_phasePin, maxPulse);
    } else if (_driverMode == DriverMode::PHASE_ENABLE) {
        analogWrite(_enablePin, 0);
        analogWrite(_phasePin, 0);
    }
    // When the motor is stopped. Set this term back to zero to reset the accumulated error.
    _iTerm = 0;
}

// When the motor is moving, "brake()" and "release()" effectively
// stops the motor from rotating. However "brake()" does it
// immideatly, whereas, "release()" coasts.
void MotorWithEncoder::release() {
    if (_driverMode == DriverMode::PWM) {
        analogWrite(_enablePin, 0);
        analogWrite(_phasePin, 0);
    } else if (_driverMode == DriverMode::PHASE_ENABLE)
        this->brake();

    // When the motor is stopped. Set this term back to zero to reset the accumulated error.
    _iTerm = 0;
}

// Set a flag to home the motor
// Home it in the update function.
void MotorWithEncoder::home() {
    // Do not home if limit switch is not set
    if (!_hasLimitSwitch) return;
    // Do not home if already homing
    if (_homing) return;
    _homing = true;
    _homingState = MOVING_TO_LIMIT_SWITCH;
}

// Non-Blocking function for homing the motor.
// This function is called in the update() function.
// This function ignores the commanded values and movement restrictions (Except Hard Stop).
void MotorWithEncoder::_home() {
    // Return if not homing
    if (!_homing) {
        // Reset the encoder position ONCE if the limit switch is pressed
        if (_limitSwitchState == HIGH && _limitSwitchStatePrev == LOW) this->resetEncoderPos();
        return;
    }

    if (hardStop) return;

    switch (_homingState) {
        case MOVING_TO_LIMIT_SWITCH:
            if (_limitSwitchState == HIGH) {
                _homingState = MOVING_TO_ZERO;
                break;
            }
            this->forceRotate(-1, 0.38);
            break;

        case MOVING_TO_ZERO:
            if (_limitSwitchState == LOW) {
                // Reset the encoder position
                this->resetEncoderPos();
                // this->setEncoderPos(400);
                _homingState = IDLE;
                break;
            }
            this->forceRotate(1, 0.36);
            break;

        case IDLE:
            this->release();
            _homing = false;
            break;
    }
}

// Direction is either -1 or 1.
// Pulse should be in range [0, 1]
// Use this method when the driver is configured as PWM.
void MotorWithEncoder::rotate(int direction, double pulse) {
    pulse = constrain(pulse, 0, 1);
    direction = constrain(direction, -1, 1);
    _pulse = direction * pulse;

    if (!_canMove) return;
    this->forceRotate(direction, pulse);
}

// Direction is either -1 or 1.
// Pulse should be in range [0, 1]
// Use this method when the driver is configured as PWM.
// This method forces the motor to rotate even if the is violating the position limits. Use with caution.
void MotorWithEncoder::forceRotate(int direction, double pulse) {
    pulse = constrain(pulse, 0, 1);
    direction = constrain(direction, -1, 1);
    switch (_driverMode) {
        case DriverMode::PWM:
            if (direction == -1) {
                analogWrite(_phasePin, 0);
                analogWrite(_enablePin, map(abs(pulse), 0, 1, 0, maxPulse));
            } else if (direction == 1) {
                analogWrite(_enablePin, 0);
                analogWrite(_phasePin, map(abs(pulse), 0, 1, 0, maxPulse));
            } else
                this->release();
            break;
        case DriverMode::PHASE_ENABLE:  // TODO
            this->release();
            break;
    }
}

// Sets the encoder position as the given value.
void MotorWithEncoder::setEncoderPos(int32_t position) {
    _positionOffset = _encoder->read() - position;
    this->release();
}

void MotorWithEncoder::setLimitSwitchPin(int8_t pin) {
    _hasLimitSwitch = true;
    _limitSwitchPin = pin;
}

// Sets the encoder position to zero.
void MotorWithEncoder::resetEncoderPos() {
    _positionOffset = 0;
    _encoder->readAndReset();
    this->resetIntegralTerm();
    _lastPositions = new int32_t[_positionWindowLength];
    this->release();
};

// Does not move the motor, only sets a target position.
// Use go() function after calling this function to move.
void MotorWithEncoder::setDesiredPosition(long positionDesired) { _positionDesired = positionDesired; }

// Does not move the motor, only sets a target velocity.
// Use go() function after calling this function to move.
void MotorWithEncoder::setDesiredVelocity(long velocityDesired) { _velocityDesired = velocityDesired; }

// Does not move the motor, only sets a target force.
// Use go() function after calling this function to move.
void MotorWithEncoder::setDesiredForce(double forceDesired) { _forceDesired = forceDesired; }

void MotorWithEncoder::setForce(double force) { _force = force; }

void MotorWithEncoder::setDeadBand(double deadBand) { _deadBand = deadBand; }

// By default PWM mode is enabled. Should be set according
// to the configuration of the motor driver.
void MotorWithEncoder::setDriverMode(DriverMode mode) { _driverMode = mode; }

// By default Position control mode is enabled.
void MotorWithEncoder::setControllerMode(ControllerMode mode) {
    _iTerm = 0;
    _controllerMode = mode;
    _PIDErrorPrevious = 0;
    memset(_lastErrors, 0, _errorWindowLength * sizeof(*_lastErrors));
};

// Set a coefficient to convert the counts of encoder into linear displacement in mm
void MotorWithEncoder::setCoefficient(double count_to_mm) { _count_to_mm = count_to_mm; }

// Sets the number of finite difference elements to be used for the velocity derivation from the position.
void MotorWithEncoder::setFDMElementCount(int count) {
    _fdmElementCount = count;
    _positionWindowLength = _fdmElementCount;
    _lastPositions = new int32_t[_positionWindowLength];
    _deltaTs = new long long[_positionWindowLength];
    _positionIndex = 0;
}

// Sets the length of the velocity window for the moving average filter.
void MotorWithEncoder::setVelocityWindowLength(int length) {
    _velocityWindowLength = length;
    _lastVelocities = new double[_velocityWindowLength];
    _velocityIndex = 0;
}

// Sets the proportional gain for the selected(_controllerMode) PID controller.
void MotorWithEncoder::setKp(double Kp) {
    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL:
            _KpPos = Kp;
            break;
        case ControllerMode::VELOCITY_CONTROL:
            _KpVel = Kp;
            break;
        case ControllerMode::FORCE_CONTROL:
            _KpFor = Kp;
            break;
    }
}

// Sets the derivative gain for  the selected(_controllerMode) PID controller.
void MotorWithEncoder::setKi(double Ki) {
    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL:
            _KiPos = Ki;
            break;
        case ControllerMode::VELOCITY_CONTROL:
            _KiVel = Ki;
            break;
        case ControllerMode::FORCE_CONTROL:
            _KiFor = Ki;
            break;
    }
    _iTerm = 0;
}

// Sets the integral gain for the selected(_controllerMode) PID controller.
void MotorWithEncoder::setKd(double Kd) {
    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL:
            _KdPos = Kd;
            break;
        case ControllerMode::VELOCITY_CONTROL:
            _KdVel = Kd;
            break;
        case ControllerMode::FORCE_CONTROL:
            _KdFor = Kd;
            break;
    }
}

// Sets the integral term of the PID controller to zero.
void MotorWithEncoder::resetIntegralTerm() { _iTerm = 0; }

// Returns the last updated position.
int32_t MotorWithEncoder::getPos() { return _position; }

// Returns the last updated velocity.
double MotorWithEncoder::getVel() { return _velocity; }

#warning "getAcc() is not tested. May not work as expected. Use with caution."
// Returns the last updated acceleration.
int32_t MotorWithEncoder::getAcc() { return _acceleration; }

// Returns the number of the phase pin.
uint8_t MotorWithEncoder::getPhasePin() { return _phasePin; }

// Returns the number of the enable pin.
uint8_t MotorWithEncoder::getEnablePin() { return _enablePin; }

MotorWithEncoder::ControllerMode MotorWithEncoder::getControllerMode() { return _controllerMode; }

MotorWithEncoder::DriverMode MotorWithEncoder::getDriverMode() { return _driverMode; }

// Returns the coefficient used to convert encoder count into mm.
double MotorWithEncoder::getCoefficient() { return _count_to_mm; }

// Return the proportional gain for the selected(_controllerMode) PID controller.
double MotorWithEncoder::getKp() {
    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL:
            return _KpPos;
        case ControllerMode::VELOCITY_CONTROL:
            return _KpVel;
        case ControllerMode::FORCE_CONTROL:
            return _KpFor;
        default:
            return 0;
    }
}

// Return the integral gain for the selected(_controllerMode) PID controller.
double MotorWithEncoder::getKi() {
    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL:
            return _KiPos;
        case ControllerMode::VELOCITY_CONTROL:
            return _KiVel;
        case ControllerMode::FORCE_CONTROL:
            return _KiFor;
        default:
            return 0;
    }
}

// Return the derivative gain for the selected(_controllerMode) PID controller.
double MotorWithEncoder::getKd() {
    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL:
            return _KdPos;
        case ControllerMode::VELOCITY_CONTROL:
            return _KdVel;
        case ControllerMode::FORCE_CONTROL:
            return _KdFor;
        default:
            return 0;
    }
}

// Sets the hard position limits for the motor.
// If the limit is violated motor brakes.
// The motor needs to be released to move past the limit.
void MotorWithEncoder::setHardPosLimit(int32_t lowerBound, int32_t upperBound) {
    _hasHardPosLimit = true;
    _lowerHardPosLimit = lowerBound;
    _upperHardPosLimit = upperBound;
}

int32_t MotorWithEncoder::getLowerHardPosLimit() { return _lowerHardPosLimit; }
int32_t MotorWithEncoder::getUpperHardPosLimit() { return _upperHardPosLimit; }

// Removes the hard limits for the motor.
void MotorWithEncoder::removeHardPosLimit() {
    this->release();
    _hasHardPosLimit = false;
}

// Sets the soft position limits for the motor.
// If the limit is violated motor coasts and stops.
void MotorWithEncoder::setSoftPosLimit(int32_t lowerBound, int32_t upperBound) {
    _hasSoftPosLimit = true;
    if (_hasHardPosLimit) {  // If hard stop range is narrower than the soft stop range, overwrite
                             // the hard limit range.
        if (lowerBound < _lowerHardPosLimit) _lowerHardPosLimit = lowerBound;
        if (upperBound > _upperHardPosLimit) _upperHardPosLimit = upperBound;
    }
    _lowerSoftPosLimit = lowerBound;
    _upperSoftPosLimit = upperBound;
}

int32_t MotorWithEncoder::getLowerSoftPosLimit() { return _lowerSoftPosLimit; }
int32_t MotorWithEncoder::getUpperSoftPosLimit() { return _upperSoftPosLimit; }

// Removes the soft limits fot the motor
void MotorWithEncoder::removeSoftPosLimit() {
    this->release();
    _hasSoftPosLimit = false;
}

#if defined(__arm__) && defined(CORE_TEENSY)
// Sets the analog write resolution bits.
void MotorWithEncoder::setResolutionBit(int resolutionBit) {
    analogWriteResolution(resolutionBit);
    MotorWithEncoder::maxPulse = pow(2, resolutionBit);
    MotorWithEncoder::resolutionBit = resolutionBit;
}

// Returns the set resolution bit.
int MotorWithEncoder::getResolutionBit() { return MotorWithEncoder::resolutionBit; }
#endif

double MotorWithEncoder::_calculatePID(double desired) {
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    err = 0;
    switch (_controllerMode) {
        case ControllerMode::POSITION_CONTROL:
            Kp = _KpPos;
            Ki = _KiPos;
            Kd = _KdPos;
            err = desired - _position;
            break;
        case ControllerMode::VELOCITY_CONTROL:
            Kp = _KpVel;
            Ki = _KiVel;
            Kd = _KdVel;
            err = desired - _velocity;
            break;
        case ControllerMode::FORCE_CONTROL:
            Kp = _KpFor;
            Ki = _KiFor;
            Kd = _KdFor;
            err = desired - _force;
            break;
    }

    double pTerm = Kp * err;
    _iTerm += Ki * err;
    double dTerm = Kd * (err - _PIDErrorPrevious);  // Calculate derivative term
    double pulse = pTerm + _iTerm + dTerm;          // Sum the terms

    // Anti windup if the integral term is too large. and the controller is in velocity mode.
    if (_controllerMode == ControllerMode::VELOCITY_CONTROL) {
        if (pulse > MotorWithEncoder::maxPulse || pulse < -MotorWithEncoder::maxPulse) _iTerm -= Ki * err;
        pulse = pTerm + _iTerm + dTerm;
    }

    float smallThreshold = 5;
    if (pulse > smallThreshold)
        pulse += _deadBand;
    else if (pulse < -smallThreshold)
        pulse -= _deadBand;

    _PIDErrorPrevious = err;

    if (pulse > MotorWithEncoder::maxPulse)
        return MotorWithEncoder::maxPulse;
    else if (pulse < -MotorWithEncoder::maxPulse)
        return -MotorWithEncoder::maxPulse;
    else
        return pulse;
}
