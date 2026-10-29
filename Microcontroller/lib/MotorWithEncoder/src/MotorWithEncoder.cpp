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
MotorWithEncoder* MotorWithEncoder::_motors[MotorWithEncoder::_maxMotors] = {};
int MotorWithEncoder::_activeMotorCount = 0;

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
    // "()" initializes the arrays with zeros.
    _lastErrors = new double[_errorWindowLength]();
    _lastVelocities = new double[_velocityWindowLength]();
    _lastPositions = new int32_t[_positionWindowLength]();
    _lastAccelerations = new double[_accelerationWindowLength]();
    _deltaTs = new long long[_positionWindowLength]();
    _motorIndex = motorCount++;
    if (_activeMotorCount < _maxMotors) {
        _motors[_activeMotorCount++] = this;
    }
}

MotorWithEncoder::~MotorWithEncoder() {
    for (int i = 0; i < _activeMotorCount; i++) {
        if (_motors[i] == this) {
            for (int j = i; j < _activeMotorCount - 1; j++) {
                _motors[j] = _motors[j + 1];
            }
            _activeMotorCount--;
            break;
        }
    }
    delete _encoder;
    delete[] _lastErrors;
    delete[] _lastVelocities;
    delete[] _lastPositions;
    delete[] _lastAccelerations;
    delete[] _deltaTs;
}

// Reads the encoder and updates the position, velocity, acceleration attributes.
// Also checks if the position limits are violated (If has a position limit)
// If a limit is violated or a stop flag is set, motor stops rotating.
// This should be called before anything else in each loop.
void MotorWithEncoder::update() {
    _driveAtUpdateStart = _lastDrivePulse;
    int32_t currentPos = _encoder->read() - _positionOffset;
    unsigned long nowMs = millis();
    int32_t stepIntoSwitch = (currentPos - _position) * _limitSwitchDirection;
    if (stepIntoSwitch > 0) {
        _lastMoveDirection = _limitSwitchDirection;
        _lastMoveToSwitchTime = nowMs;
    } else if (stepIntoSwitch < 0) {
        _lastMoveDirection = -_limitSwitchDirection;
    }
    double driveIntoSwitch = _driveAtUpdateStart * _limitSwitchDirection;
    if (driveIntoSwitch > 0) {
        _lastDriveDirection = _limitSwitchDirection;
        _lastDriveToSwitchTime = nowMs;
    } else if (driveIntoSwitch < 0) {
        _lastDriveDirection = -_limitSwitchDirection;
    }
    _position = currentPos;
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

    // Calculate the acceleration using backward finite difference
    // Coefficients are calculated using the link below
    // https://web.media.mit.edu/~crtaylor/calculator.html
    // The formula needs the last 5 positions, so the acceleration is 0 if fewer positions are kept.
    int64_t instantaneousAcceleration = 0;
    if (_positionWindowLength >= 5)
        instantaneousAcceleration =
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

    bool upperSoftViolation = _hasSoftPosLimit && _position >= _upperSoftPosLimit;
    bool lowerSoftViolation = _hasSoftPosLimit && _position <= _lowerSoftPosLimit;
    bool upperHardViolation = _hasHardPosLimit && _position >= _upperHardPosLimit;
    bool lowerHardViolation = _hasHardPosLimit && _position <= _lowerHardPosLimit;

    _softLimitViolation = upperSoftViolation || lowerSoftViolation;
    _hardLimitViolation = upperHardViolation || lowerHardViolation;

    bool upperLimitViolation = upperSoftViolation || upperHardViolation;
    bool lowerLimitViolation = lowerSoftViolation || lowerHardViolation;
    _isOppositeLimitViolation = (_limitSwitchDirection < 0) ? upperLimitViolation : lowerLimitViolation;

    if (_softLimitViolation || _hardLimitViolation) {
        if (upperLimitViolation && _pulse > 0)
            _isDirectionSafeToMove = false;
        else if (upperLimitViolation && _pulse <= 0)
            _isDirectionSafeToMove = true;
        else if (!upperLimitViolation && _pulse < 0)
            _isDirectionSafeToMove = false;
        else if (!upperLimitViolation && _pulse >= 0)
            _isDirectionSafeToMove = true;
    }

    // While homing, bypass only the switch-end position limit; keep the opposite limit active.
    bool enforcePosLimit = !_homing || _isOppositeLimitViolation;

    if (hardStop || (enforcePosLimit && _hardLimitViolation && !_isDirectionSafeToMove))
        this->brake();
    else if (softStop || (enforcePosLimit && _softLimitViolation && !_isDirectionSafeToMove))
        this->release();

    if (_softLimitViolation || _hardLimitViolation)
        _canMove = _isDirectionSafeToMove && !hardStop && !softStop;
    else
        _canMove = !(hardStop || softStop);

    // While another motor on the shared limit switch pin is homing, keep this motor still, so that every press of
    // the switch belongs to the homing motor. Braking also resists the user pushing it into its switch.
    if (isSharedLimitSwitchBusy()) {
        this->brake();
        _canMove = false;
    }

    if (!_canMove) this->resetIntegralTerm();

    // Debounce the limit switch if it is present and infer which switch was pressed if shared
    _updateLimitSwitch();
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

    // In force control the output is kept while the force is steady. The caller releases the motor.
    bool keepOutput = _driverMode == DriverMode::PWM && _isSteadyState && _controllerMode == FORCE_CONTROL;
    if (!keepOutput) _drive(_pulse);

    if (commandedValue == 0)                                  // Avoid division by zero
        _lastErrors[errorIndex] = -valueToCalculateErrorFor;  // Calculate percent error
    else
        _lastErrors[errorIndex] = (commandedValue - valueToCalculateErrorFor) / (commandedValue);

    errorIndex = (errorIndex + 1) % _errorWindowLength;
    if (_errorCount < _errorWindowLength) _errorCount++;

    double sum = 0;
    for (int i = 0; i < _errorWindowLength; i++) sum += abs(_lastErrors[i]);

    averageError = sum / (double)_errorWindowLength;

    // Steady state is reached if the error is less than 2 percent. The window has to be full first, otherwise the
    // zeros left by resetController() make the error look smaller than it is.
    if (_errorCount == _errorWindowLength && averageError <= 0.02) {
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

// Position control that keeps its own gains (setHoldGains) and PID state, so it can be used for a moment without
// changing the controller mode and resetting the state of its controller.
// HapticDevice uses it to stop the finger at a rigid virtual object while in force control.
// Call resetHold() before starting a new hold.
void MotorWithEncoder::holdPosition(int32_t position) {
    if (_homing) return;

    double holdError = position - _position;
    _holdITerm += _KiHold * holdError;
    double dTerm = _holdSkipDerivative ? 0 : _KdHold * (holdError - _holdErrorPrevious);
    _holdSkipDerivative = false;
    _holdErrorPrevious = holdError;

    _pulse = _addDeadBandAndLimit(_KpHold * holdError + _holdITerm + dTerm);

    if (!_canMove) return;
    _drive(_pulse);
}

// Drives the motor with the given pulse. Releases it if the pulse is too small to move it.
void MotorWithEncoder::_drive(double pulse) {
    switch (_driverMode) {
        case DriverMode::PWM:
            if (pulse > 5) {
                _lastDrivePulse = pulse;
                analogWrite(_phasePin, abs(pulse));
                analogWrite(_enablePin, 0);
            } else if (pulse < -5) {
                _lastDrivePulse = pulse;
                analogWrite(_phasePin, 0);
                analogWrite(_enablePin, abs(pulse));
            } else {
                this->release();
            }
            break;
        case DriverMode::PHASE_ENABLE:  // TODO
            this->release();
            break;
    }
}

// When the motor is moving, "brake()" and "release()"
// effectively stops the motor from rotating.
// However "brake()" does it immideatly, whereas, "release()" coasts.
void MotorWithEncoder::brake() {
    _lastDrivePulse = 0;
    if (_driverMode == DriverMode::PWM) {
        analogWrite(_enablePin, maxPulse);
        analogWrite(_phasePin, maxPulse);
    } else if (_driverMode == DriverMode::PHASE_ENABLE) {
        analogWrite(_enablePin, 0);
        analogWrite(_phasePin, 0);
    }
    // When the motor is stopped. Set this term back to zero to reset the accumulated error.
    this->resetIntegralTerm();
}

// When the motor is moving, "brake()" and "release()" effectively
// stops the motor from rotating. However "brake()" does it
// immideatly, whereas, "release()" coasts.
void MotorWithEncoder::release() {
    _lastDrivePulse = 0;
    if (_driverMode == DriverMode::PWM) {
        analogWrite(_enablePin, 0);
        analogWrite(_phasePin, 0);
    } else if (_driverMode == DriverMode::PHASE_ENABLE)
        this->brake();

    // When the motor is stopped. Set this term back to zero to reset the accumulated error.
    this->resetIntegralTerm();
}

// Set a flag to home the motor
// Home it in the update function.
// Call it from the same context as update() (not from an interrupt), because it changes the homing state of the
// other motors on the shared limit switch pin too.
void MotorWithEncoder::home() {
    // Do not home if limit switch is not set
    if (!_hasLimitSwitch) return;
    // Do not home if already homing. A motor that is only clearing the switch for another motor homes afterwards.
    if (_homing) {
        if (_homingState == CLEARING_SHARED_SWITCH) {
            _resumeHomingAfterClearing = true;
            _homingFailed = false;
        }
        return;
    }

    unsigned long now = millis();
    _triedClearingOtherMotor = false;
    _resumeHomingAfterClearing = false;
    _homingFailed = false;

    // If another motor sharing the same limit switch pin is already homing or holding the switch HIGH,
    // serialize homing so only one motor uses the shared switch at a time.
    for (int i = 0; i < _activeMotorCount; i++) {
        MotorWithEncoder* other = _motors[i];
        if (other == this || !other->_hasLimitSwitch || other->_limitSwitchPin != _limitSwitchPin) continue;
        if (other->_homing) {
            _homing = true;
            _homingState = WAITING_FOR_SHARED_SWITCH;
            _homingStateStartTime = now;
            return;
        }
        if (other->_limitSwitchState == HIGH) {
            // The other motor is currently resting on its limit switch, holding the parallel line HIGH.
            // Back the other motor off its switch first so the shared pin returns to LOW.
            other->_homing = true;
            other->_homingState = CLEARING_SHARED_SWITCH;
            other->_homingStateStartTime = now;
            other->_resumeHomingAfterClearing = false;
            _homing = true;
            _homingState = WAITING_FOR_SHARED_SWITCH;
            _homingStateStartTime = now;
            return;
        }
    }

    _homing = true;
    _homingState = MOVING_TO_LIMIT_SWITCH;
    _homingStateStartTime = now;
}

// Returns true if the last homing of this motor gave up (see _abortSharedHoming). Cleared when homing starts again.
bool MotorWithEncoder::homingFailed() { return _homingFailed; }

bool MotorWithEncoder::isHoming() { return _homing; }

bool MotorWithEncoder::isLimitSwitchPressed() { return _limitSwitchState; }

// Returns true if this motor is not homing but another motor sharing its limit switch pin is.
// The motor is kept released until the other motor finishes homing.
bool MotorWithEncoder::isSharedLimitSwitchBusy() {
    if (!_hasLimitSwitch || _homing) return false;
    for (int i = 0; i < _activeMotorCount; i++) {
        MotorWithEncoder* other = _motors[i];
        if (other != this && other->_hasLimitSwitch && other->_limitSwitchPin == _limitSwitchPin && other->_homing)
            return true;
    }
    return false;
}

// Sets the encoder position corresponding to this motor's limit switch location:
// 0 when the switch is at the lower end (-1), or Upper Soft/Hard Limit when at the upper end (+1).
void MotorWithEncoder::_applyLimitSwitchHomePosition() {
    if (_limitSwitchDirection > 0) {
        if (_hasSoftPosLimit) {
            this->setEncoderPos(_upperSoftPosLimit);
            return;
        }
        if (_hasHardPosLimit) {
            this->setEncoderPos(_upperHardPosLimit);
            return;
        }
    }
    this->resetEncoderPos();
}

// Releases and aborts homing on this motor and any other motor sharing the same limit switch pin.
// Marks the motors that were homing for themselves as failed; a motor that was only clearing the switch for another
// motor was not being homed, so its position is still as good as before.
void MotorWithEncoder::_abortSharedHoming() {
    for (int i = 0; i < _activeMotorCount; i++) {
        MotorWithEncoder* m = _motors[i];
        if (m->_hasLimitSwitch && m->_limitSwitchPin == _limitSwitchPin && m->_homing) {
            m->_homingFailed = m->_homingState != CLEARING_SHARED_SWITCH || m->_resumeHomingAfterClearing;
            m->_resumeHomingAfterClearing = false;
            m->release();
            m->_homing = false;
            m->_homingState = IDLE;
        }
    }
}

bool MotorWithEncoder::_isActivelyHoming() {
    return _homing && (_homingState == MOVING_TO_LIMIT_SWITCH || _homingState == MOVING_TO_ZERO ||
                       _homingState == CLEARING_SHARED_SWITCH);
}

bool MotorWithEncoder::_hasEncoderMotionTowardSwitch(unsigned long now) {
    int32_t livePos = _encoder->read() - _positionOffset;
    bool liveStepTowardSwitch = ((livePos - _position) * _limitSwitchDirection) > 0;
    bool recentStepTowardSwitch = (_lastMoveDirection == _limitSwitchDirection) && (now - _lastMoveToSwitchTime <= 250);
    bool velTowardSwitch = (_velocity * _limitSwitchDirection) > 0;
    return liveStepTowardSwitch || recentStepTowardSwitch || velTowardSwitch;
}

bool MotorWithEncoder::_isMovingTowardLimitSwitch(unsigned long now) {
    bool liveDriveTowardSwitch = ((_lastDrivePulse * _limitSwitchDirection) > 0) ||
                                 ((_driveAtUpdateStart * _limitSwitchDirection) > 0);
    bool recentDriveTowardSwitch = (_lastDriveDirection == _limitSwitchDirection) && (now - _lastDriveToSwitchTime <= 250);
    return _hasEncoderMotionTowardSwitch(now) || liveDriveTowardSwitch || recentDriveTowardSwitch;
}

double MotorWithEncoder::_distanceFromSwitchLimit() {
    int32_t livePos = _encoder->read() - _positionOffset;
    int32_t switchPos = 0;
    if (_limitSwitchDirection > 0) {
        if (_hasSoftPosLimit)
            switchPos = _upperSoftPosLimit;
        else if (_hasHardPosLimit)
            switchPos = _upperHardPosLimit;
    }
    double distCounts = (_limitSwitchDirection < 0) ? (double)max<int32_t>(0, livePos - switchPos)
                                                    : (double)max<int32_t>(0, switchPos - livePos);
    if (_hasSoftPosLimit && _upperSoftPosLimit > _lowerSoftPosLimit) {
        return distCounts / (double)(_upperSoftPosLimit - _lowerSoftPosLimit);
    }
    if (_hasHardPosLimit && _upperHardPosLimit > _lowerHardPosLimit) {
        return distCounts / (double)(_upperHardPosLimit - _lowerHardPosLimit);
    }
    return fabs((double)(livePos - switchPos));
}

// Infers which motor's limit switch is pressed when multiple switches are wired in parallel to a single input pin.
MotorWithEncoder* MotorWithEncoder::_inferLimitSwitchOwner(unsigned long now) {
    // 1. If a motor on this pin is actively homing, the press belongs to it. The other motors on the pin are kept
    // still while it homes (see isSharedLimitSwitchBusy), so they cannot be the ones pressing.
    for (int i = 0; i < _activeMotorCount; i++) {
        MotorWithEncoder* m = _motors[i];
        if (m->_hasLimitSwitch && m->_limitSwitchPin == _limitSwitchPin && m->_isActivelyHoming()) {
            return m;
        }
    }

    // 2. If a motor on this pin already claimed the current press while the pin stays HIGH, keep that owner.
    for (int i = 0; i < _activeMotorCount; i++) {
        MotorWithEncoder* m = _motors[i];
        if (m->_hasLimitSwitch && m->_limitSwitchPin == _limitSwitchPin && m->_limitSwitchState == HIGH) {
            return m;
        }
    }

    // 3. Rising edge during normal operation: infer which motor just hit its switch based on motion toward the
    // switch (encoder motion or motor drive in _limitSwitchDirection) and proximity to its switch limit.
    MotorWithEncoder* best = this;
    for (int i = 0; i < _activeMotorCount; i++) {
        MotorWithEncoder* m = _motors[i];
        if (m == best || !m->_hasLimitSwitch || m->_limitSwitchPin != _limitSwitchPin) continue;

        bool mMoving = m->_isMovingTowardLimitSwitch(now);
        bool bestMoving = best->_isMovingTowardLimitSwitch(now);
        if (mMoving != bestMoving) {
            if (mMoving) best = m;
            continue;
        }

        if (mMoving && bestMoving) {
            bool mEncMoved = m->_hasEncoderMotionTowardSwitch(now);
            bool bestEncMoved = best->_hasEncoderMotionTowardSwitch(now);
            if (mEncMoved != bestEncMoved) {
                if (mEncMoved) best = m;
                continue;
            }
        }

        double mDist = m->_distanceFromSwitchLimit();
        double bestDist = best->_distanceFromSwitchLimit();
        if (mDist < bestDist) {
            best = m;
        }
    }

    for (int i = 0; i < _activeMotorCount; i++) {
        MotorWithEncoder* m = _motors[i];
        if (m->_hasLimitSwitch && m->_limitSwitchPin == _limitSwitchPin) {
            m->_rawLimitSwitchState = HIGH;
            m->_limitSwitchState = (m == best);
        }
    }
    return best;
}

void MotorWithEncoder::_updateLimitSwitch() {
    if (!_hasLimitSwitch) return;

    unsigned long now = millis();
    _limitSwitchReading = digitalRead(_limitSwitchPin);
    if (_limitSwitchReading != _limitSwitchReadingPrev) _limitSwitchDebounceTime = now;
    if ((now - _limitSwitchDebounceTime) > 30) _rawLimitSwitchState = _limitSwitchReading;
    _limitSwitchReadingPrev = _limitSwitchReading;

    if (_rawLimitSwitchState == LOW) {
        _limitSwitchState = LOW;
        if (_limitSwitchReading == LOW) {
            for (int i = 0; i < _activeMotorCount; i++) {
                MotorWithEncoder* m = _motors[i];
                if (m != this && m->_hasLimitSwitch && m->_limitSwitchPin == _limitSwitchPin &&
                    (now - m->_limitSwitchDebounceTime) > 30) {
                    m->_rawLimitSwitchState = LOW;
                    m->_limitSwitchState = LOW;
                }
            }
        }
        return;
    }

    MotorWithEncoder* owner = _inferLimitSwitchOwner(now);
    _limitSwitchState = (owner == this);
}

// Non-Blocking function for homing the motor.
// This function is called in the update() function.
// This function ignores the commanded values and movement restrictions (Except Hard Stop and opposite limit).
void MotorWithEncoder::_home() {
    // Return if not homing
    if (!_homing) {
        // Set the encoder position ONCE if this motor's limit switch is inferred to be pressed
        if (_limitSwitchState == HIGH && _limitSwitchStatePrev == LOW) _applyLimitSwitchHomePosition();
        return;
    }

    unsigned long now = millis();
    if (hardStop || softStop) {
        _homingStateStartTime = now;
        return;
    }

    switch (_homingState) {
        case WAITING_FOR_SHARED_SWITCH: {
            this->brake();
            bool otherActivelyHoming = false;
            for (int i = 0; i < _activeMotorCount; i++) {
                MotorWithEncoder* other = _motors[i];
                if (other != this && other->_hasLimitSwitch && other->_limitSwitchPin == _limitSwitchPin &&
                    other->_isActivelyHoming()) {
                    otherActivelyHoming = true;
                    break;
                }
            }
            if (!otherActivelyHoming && _rawLimitSwitchState == LOW) {
                _homingState = MOVING_TO_LIMIT_SWITCH;
                _homingStateStartTime = now;
            }
        } break;

        case CLEARING_SHARED_SWITCH:
            if (_rawLimitSwitchState == LOW) {
                // Do not reset the encoder position when clearing on behalf of another motor
                this->release();
                if (_resumeHomingAfterClearing) {
                    // This motor was waiting to home itself, so it waits for its turn again
                    _resumeHomingAfterClearing = false;
                    _homingState = WAITING_FOR_SHARED_SWITCH;
                    _homingStateStartTime = now;
                } else {
                    _homing = false;
                    _homingState = IDLE;
                }
                break;
            }
            if (_isOppositeLimitViolation || (now - _homingStateStartTime) > 500) {
                _abortSharedHoming();
                break;
            }
            this->forceRotate(-_limitSwitchDirection, 0.36);
            break;

        case MOVING_TO_LIMIT_SWITCH:
            if (_limitSwitchState == HIGH) {
                _homingState = MOVING_TO_ZERO;
                _homingStateStartTime = now;
                break;
            }
            this->forceRotate(_limitSwitchDirection, 0.38);
            break;

        case MOVING_TO_ZERO:
            if (_limitSwitchState == LOW) {
                _applyLimitSwitchHomePosition();
                this->release();
                _homing = false;
                _homingState = IDLE;
                break;
            }
            if (_isOppositeLimitViolation) {
                _abortSharedHoming();
                break;
            }
            // If backing off for more than 500 ms does not release a shared parallel switch, the other motor on this
            // pin may be holding its switch closed (e.g. from before power-on). Try clearing the other motor once for
            // at most 500 ms; if already tried or no other motor shares the pin, abort homing.
            if ((now - _homingStateStartTime) > 500) {
                if (!_triedClearingOtherMotor) {
                    for (int i = 0; i < _activeMotorCount; i++) {
                        MotorWithEncoder* other = _motors[i];
                        if (other != this && other->_hasLimitSwitch && other->_limitSwitchPin == _limitSwitchPin) {
                            _triedClearingOtherMotor = true;
                            this->release();
                            _homingState = WAITING_FOR_SHARED_SWITCH;
                            _homingStateStartTime = now;
                            // If the other motor was waiting to home itself, it homes after clearing the switch
                            other->_resumeHomingAfterClearing = other->_homing;
                            other->_homing = true;
                            other->_homingState = CLEARING_SHARED_SWITCH;
                            other->_homingStateStartTime = now;
                            return;
                        }
                    }
                }
                _abortSharedHoming();
                break;
            }
            this->forceRotate(-_limitSwitchDirection, 0.36);
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
                _lastDrivePulse = -map(abs(pulse), 0, 1, 0, maxPulse);
                analogWrite(_phasePin, 0);
                analogWrite(_enablePin, map(abs(pulse), 0, 1, 0, maxPulse));
            } else if (direction == 1) {
                _lastDrivePulse = map(abs(pulse), 0, 1, 0, maxPulse);
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
    _position = position;
    _positionPrevious = position;
    _lastMoveDirection = 0;
    _lastDriveDirection = 0;
    this->resetIntegralTerm();
    _resetPositionHistory(position);
    this->release();
}

void MotorWithEncoder::setLimitSwitchPin(int8_t pin, int direction) {
    _hasLimitSwitch = true;
    _limitSwitchPin = pin;
    _limitSwitchDirection = (direction >= 0) ? 1 : -1;
}

int MotorWithEncoder::getLimitSwitchDirection() { return _limitSwitchDirection; }

// Sets the encoder position to zero.
void MotorWithEncoder::resetEncoderPos() {
    _positionOffset = 0;
    _encoder->readAndReset();
    _position = 0;
    _positionPrevious = 0;
    _lastMoveDirection = 0;
    _lastDriveDirection = 0;
    this->resetIntegralTerm();
    _resetPositionHistory(0);
    this->release();
};

// Fills the position history with the given position, so that the velocity does not jump when the position is
// changed by resetting or setting the encoder position.
void MotorWithEncoder::_resetPositionHistory(int32_t position) {
    for (int i = 0; i < _positionWindowLength; i++) _lastPositions[i] = position;
}

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
    _controllerMode = mode;
    this->resetController();
};

// Clears the state of the controller: the integral term, the previous error of the derivative term and the errors
// used to detect the steady state.
void MotorWithEncoder::resetController() {
    _iTerm = 0;
    _PIDErrorPrevious = 0;
    _skipDerivative = true;  // The previous error is unknown, so the first derivative would be a spike
    memset(_lastErrors, 0, _errorWindowLength * sizeof(*_lastErrors));
    _errorCount = 0;
    _isSteadyState = false;
}

// Clears the state of holdPosition().
void MotorWithEncoder::resetHold() {
    _holdITerm = 0;
    _holdErrorPrevious = 0;
    _holdSkipDerivative = true;
}

// Set a coefficient to convert the counts of encoder into linear displacement in mm
void MotorWithEncoder::setCoefficient(double count_to_mm) { _count_to_mm = count_to_mm; }

// Sets the number of finite difference elements to be used for the velocity derivation from the position.
void MotorWithEncoder::setFDMElementCount(int count) {
    _fdmElementCount = count < 2 ? 2 : count;
    _positionWindowLength = _fdmElementCount;
    delete[] _lastPositions;
    delete[] _deltaTs;
    _lastPositions = new int32_t[_positionWindowLength]();
    _deltaTs = new long long[_positionWindowLength]();
    _positionIndex = 0;
    _resetPositionHistory(_position);
}

// Sets the length of the velocity window for the moving average filter.
void MotorWithEncoder::setVelocityWindowLength(int length) {
    _velocityWindowLength = length < 1 ? 1 : length;
    delete[] _lastVelocities;
    _lastVelocities = new double[_velocityWindowLength]();
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

// Scales the proportional gain of the force controller without changing the gain itself.
// HapticDevice uses this to lower the gain while the finger is in contact with a virtual object.
void MotorWithEncoder::setForceKpScale(double scale) { _forceKpScale = scale; }

// Sets the gains of holdPosition().
void MotorWithEncoder::setHoldGains(double Kp, double Ki, double Kd) {
    _KpHold = Kp;
    _KiHold = Ki;
    _KdHold = Kd;
    _holdITerm = 0;
}

// Sets the integral terms of the PID controller and of holdPosition() to zero.
void MotorWithEncoder::resetIntegralTerm() {
    _iTerm = 0;
    _holdITerm = 0;
}

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
double MotorWithEncoder::getKp() { return getKp(_controllerMode); }

// Return the integral gain for the selected(_controllerMode) PID controller.
double MotorWithEncoder::getKi() { return getKi(_controllerMode); }

// Return the derivative gain for the selected(_controllerMode) PID controller.
double MotorWithEncoder::getKd() { return getKd(_controllerMode); }

// Return the proportional gain for the PID controller of the given mode.
double MotorWithEncoder::getKp(ControllerMode mode) {
    switch (mode) {
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

// Return the integral gain for the PID controller of the given mode.
double MotorWithEncoder::getKi(ControllerMode mode) {
    switch (mode) {
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

// Return the derivative gain for the PID controller of the given mode.
double MotorWithEncoder::getKd(ControllerMode mode) {
    switch (mode) {
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
    MotorWithEncoder::maxPulse = (1 << resolutionBit) - 1;
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
            Kp = _KpFor * _forceKpScale;
            Ki = _KiFor;
            Kd = _KdFor;
            err = desired - _force;
            break;
    }

    double pTerm = Kp * err;
    _iTerm += Ki * err;
    double dTerm = _skipDerivative ? 0 : Kd * (err - _PIDErrorPrevious);  // Calculate derivative term
    _skipDerivative = false;
    double pulse = pTerm + _iTerm + dTerm;  // Sum the terms

    // Anti windup if the integral term is too large. and the controller is in velocity mode.
    if (_controllerMode == ControllerMode::VELOCITY_CONTROL) {
        if (pulse > MotorWithEncoder::maxPulse || pulse < -MotorWithEncoder::maxPulse) _iTerm -= Ki * err;
        pulse = pTerm + _iTerm + dTerm;
    }

    _PIDErrorPrevious = err;
    return _addDeadBandAndLimit(pulse);
}

// Adds the dead band to the pulse and limits it to the valid range of the analogWrite function.
double MotorWithEncoder::_addDeadBandAndLimit(double pulse) {
    float smallThreshold = 5;
    if (pulse > smallThreshold)
        pulse += _deadBand;
    else if (pulse < -smallThreshold)
        pulse -= _deadBand;

    if (pulse > MotorWithEncoder::maxPulse)
        return MotorWithEncoder::maxPulse;
    else if (pulse < -MotorWithEncoder::maxPulse)
        return -MotorWithEncoder::maxPulse;
    else
        return pulse;
}
