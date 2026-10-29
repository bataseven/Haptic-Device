#include "HapticDevice.h"

// Initialize the motors
MotorWithEncoder *HapticDevice::finger =
    new MotorWithEncoder(8, 9, 3, 4); // (PHASE_PIN, ENABLE_PIN, ENCODER_A, ENCODER_B)
// MotorWithEncoder* HapticDevice::finger =
//     new MotorWithEncoder(14, 15, 21, 20);  // (PHASE_PIN, ENABLE_PIN, ENCODER_A, ENCODER_B)
MotorWithEncoder *HapticDevice::palm =
    new MotorWithEncoder(14, 15, 21, 20); // (PHASE_PIN, ENABLE_PIN, ENCODER_A, ENCODER_B)
// MotorWithEncoder* HapticDevice::palm =
//     new MotorWithEncoder(8, 9, 3, 4);  // (PHASE_PIN, ENABLE_PIN, ENCODER_A, ENCODER_B)

HapticDevice::HapticDevice() {
    // Initialize the LED when the object is created.
    led.begin();
}

void HapticDevice::initialize() {
    thumbInnerSensor = new PressureSensor(A9);
    thumbOuterSensor = new PressureSensor(A8);
    indexInnerSensor = new PressureSensor(A3);
    indexOuterSensor = new PressureSensor(A2);

    // Setup the finger motor parameters
    finger->setDriverMode(MotorWithEncoder::DriverMode::PWM);
    finger->setControllerMode(MotorWithEncoder::ControllerMode::POSITION_CONTROL);

    // finger->setKp(EEPROM.read(address));
    finger->setKp(1);
    finger->setKi(0);
    finger->setKd(6);

    // ! TODO - These are not tuned
    finger->setFDMElementCount(5);
    finger->setControllerMode(MotorWithEncoder::ControllerMode::VELOCITY_CONTROL);
    finger->setKp(0.0375);
    finger->setKi(0.00001);
    finger->setKd(0.03);

    finger->setControllerMode(MotorWithEncoder::ControllerMode::FORCE_CONTROL);
    finger->setKp(FINGER_FORCE_KP);
    finger->setKi(5);
    finger->setKd(80);

    finger->setHoldGains(RIGID_BODY_KP, RIGID_BODY_KI, RIGID_BODY_KD);

    finger->setResolutionBit(13);
    finger->setDeadBand(725);
    finger->setSoftPosLimit(FINGER_LOWER_SOFT_LIMIT, FINGER_UPPER_SOFT_LIMIT);
    // finger->setCoefficient(COUNT_TO_MM);
    finger->setControllerMode(MotorWithEncoder::ControllerMode::POSITION_CONTROL);
    finger->setControllerMode(MotorWithEncoder::ControllerMode::FORCE_CONTROL);

    // Setup the palm motor parameters
    palm->setDriverMode(MotorWithEncoder::DriverMode::PWM);
    palm->setControllerMode(MotorWithEncoder::ControllerMode::VELOCITY_CONTROL);
    palm->setKp(5);
    palm->setKi(0.01);
    palm->setKd(5);

    palm->setControllerMode(MotorWithEncoder::ControllerMode::POSITION_CONTROL);
    palm->setKp(30);
    palm->setKi(0.001);
    palm->setKd(785);

    palm->setResolutionBit(13);
    palm->setDeadBand(2275);

    // Set the frequencies for the motors pins to a higher value for a quieter operation.
    analogWriteFrequency(finger->getEnablePin(), PWM_FREQUENCY);
    analogWriteFrequency(finger->getPhasePin(), PWM_FREQUENCY);

    analogWriteFrequency(palm->getEnablePin(), PWM_FREQUENCY);
    analogWriteFrequency(palm->getPhasePin(), PWM_FREQUENCY);

    // Setup the waveformgenerator for the finger
    waveformGeneratorFinger.setType(WaveformGenerator::WaveType::CONSTANT);
    waveformGeneratorFinger.setFrequency(0);
    waveformGeneratorFinger.setOffset(0);
    waveformGeneratorFinger.setAmplitude(0);

    // Setup the waveformgenerator for the palm
    waveformGeneratorPalm.setType(WaveformGenerator::WaveType::CONSTANT);
    waveformGeneratorPalm.setFrequency(0);
    waveformGeneratorPalm.setOffset(0);
    waveformGeneratorPalm.setAmplitude(0);
    waveformGeneratorPalm.setFourierTerms(125);

    fingerButton = new FunctionalButton(fingerResetPin);
    palmButton = new FunctionalButton(palmResetPin);

    // Initialize I2C communication
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock speed to 400kHz

    // Initialization of the BNO055
    // BNO_Init(&_BNO055);  // Assigning the structure to hold information about the device

    // Configuration to NDoF mode (Currently defaulted to NDoF)
    // bno055_set_operation_mode(OPERATION_MODE_NDOF);

    // GPIO2 is shared by the finger and palm limit switches (wired in parallel)
    finger->setLimitSwitchPin(2, FINGER_LIMIT_SWITCH_DIRECTION);
    palm->setLimitSwitchPin(2, PALM_LIMIT_SWITCH_DIRECTION);
    palm->setSoftPosLimit(PALM_LOWER_SOFT_LIMIT, PALM_UPPER_SOFT_LIMIT);
    palm->setHardPosLimit(PALM_LOWER_HARD_LIMIT, PALM_UPPER_HARD_LIMIT);
    palm->home();
    finger->home();
}

void HapticDevice::_LEDUpdate() {
    if (!this->serial || !this->serial->availableForWrite()) { // If the serial port is not open, light up orange
        led.setPixelColor(0, ORANGE);
        _currentColor = ORANGE;
    } else { // If the serial port is open, light up green (Normal contactless operation)
        led.setPixelColor(0, GREEN);
        _currentColor = GREEN;
    }

    if (hostTimedOut) { // If the motors were stopped because the host stopped sending messages, light up purple
        led.setPixelColor(0, PURPLE);
        _currentColor = PURPLE;
    }

    if (finger->homingFailed() || palm->homingFailed()) { // If homing gave up because the limit switch did not
                                                          // release, light up red until the motor is homed again
        led.setPixelColor(0, RED);
        _currentColor = RED;
    }

    if (obj.inContact) { // If an object is in contact, light up blue
        led.setPixelColor(0, BLUE);
        _currentColor = BLUE;
    }

    if (fingerButtonState == FunctionalButton::DoubleHold ||
        palmButtonState == FunctionalButton::DoubleHold) { // If either button is double held, turn off the LED
        led.setPixelColor(0, BLACK);
        _currentColor = BLACK;
    }

    if (millis() - _encoderResetLEDTimer <
        500) { // If the encoder was reset, light up pink for 500ms (Single press of either button)
        led.setPixelColor(0, PINK);
        _currentColor = PINK;
    }

    if (millis() - _imuLEDTimer <
        2000) { // If the MPU6050 is available and activated, light up white for 2000ms (Single hold of both buttons)
        if (imuActive) {
            led.setPixelColor(0, CYAN);
            _currentColor = CYAN;
        } else { // If the MPU6050 was deactivated, light up red for 2000ms (Single hold of both buttons)
            led.setPixelColor(0, RED);
            _currentColor = RED;
        }
    } else if ((fingerButtonState == FunctionalButton::SingleHold && !palmButton->read()) ||
               (palmButtonState == FunctionalButton::SingleHold &&
                !fingerButton->read())) { // If either button is single held, light up white
        led.setPixelColor(0, WHITE);
        _currentColor = WHITE;
    }
    // Pin 13 (the built-in LED) is not set here. loop() blinks it to show that the program is running.

    if (_currentColor != _lastColor) {
        // Keep track of the last color and change the LED only when the color changes.
        // This prevents calling the show() function too often.
        led.show();
        _lastColor = _currentColor;
    }
}

void HapticDevice::update() {
    palm->update();   // Update the attributes of the motor such as the velocity and position.
    finger->update(); // Update the attributes of the motor such as the velocity and position.

    thumbInnerSensor
        ->update(); // Update the sensor readings and save it to a window that will be used for moving average filter.
    thumbOuterSensor
        ->update(); // Update the sensor readings and save it to a window that will be used for moving average filter.
    indexInnerSensor
        ->update(); // Update the sensor readings and save it to a window that will be used for moving average filter.
    indexOuterSensor
        ->update(); // Update the sensor readings and save it to a window that will be used for moving average filter.

    generatedFingerWave = waveformGeneratorFinger.generate(); // Generate the waveform for the finger.
    generatedPalmWave = waveformGeneratorPalm.generate();     // Generate the waveform for the palm.

    _LEDUpdate();

    fingerButton->update();                       // Update the button state.
    fingerButtonState = fingerButton->getState(); // Get the button state.

    palmButton->update();                     // Update the button state.
    palmButtonState = palmButton->getState(); // Get the button state.

    if (fingerButtonState == FunctionalButton::SingleClick) {
        _encoderResetLEDTimer = millis();
        finger->resetEncoderPos();
    }
    if (palmButtonState == FunctionalButton::SingleClick) {
        _encoderResetLEDTimer = millis();
        palm->resetEncoderPos();
    }
    if (palmButtonState == FunctionalButton::DoubleHold && !fingerButton->read()) { // Move down to the limit switch
        noInterrupts(); // home() also changes the finger's state, which the control loop interrupt uses
        palm->home();
        interrupts();
    }

    if (fingerButtonState == FunctionalButton::SingleHold && palmButtonState == FunctionalButton::SingleHold &&
        !_imuActivationChanged) {
        _imuLEDTimer = millis();
        imuActive = !imuActive;
        _imuActivationChanged = true;
    }

    if (fingerButtonState == FunctionalButton::None &&
        palmButtonState == FunctionalButton::None) { // Release both buttons to reset the flag, so that the MPU can be
                                                     // activated again.
        _imuActivationChanged = false;
    }

    // Measure the time
    // unsigned long currentMillis = micros();
    if (imuActive && millis() - _imuUpdateTimer > 1000.0 / _imuUpdateFrequency) {
        _imuUpdateTimer = millis();
        // bno055_read_euler_hrp(&eulerAngles);  // Update Euler data into the structure
    }
    // Serial.println("BNO:" + String(micros() - currentMillis));
}

void HapticDevice::fingerControlLoop() {
    // Measure the time
    // static unsigned long prev = micros();
    // Serial.println("Loop:" + String(micros() - prev));
    // prev = micros();
    if (finger->getPos() > (FINGER_UPPER_SOFT_LIMIT - obj.nominalWidth)) {
        float penetrationDepth = finger->getPos() - (FINGER_UPPER_SOFT_LIMIT - obj.nominalWidth);
        float springForce = -(obj.stiffness * penetrationDepth * COUNT_TO_MM);
        float dampingForce = -(obj.damping * finger->getVel() * COUNT_TO_MM);
        desiredFingerForce = obj.stiffness < RIGID_BODY_STIFFNESS_THRESHOLD ? springForce + dampingForce : 0;
        // Only scales the gain, so that the Kp set over serial is kept.
        finger->setForceKpScale(FINGER_CONTACT_KP_SCALE);
        obj.inContact = true;
    } else {
        // Reset the integral term of the PID controller when the finger is no longer in contact with the object.
        // This prevents the finger to bounce when the object is removed.
        if (obj.inContactPrev)
            finger->resetIntegralTerm();
        desiredFingerForce = 0;
        obj.inContact = false;
        finger->setForceKpScale(1);
    }

    double outerForceScale = 1; // This is higher because I want the user to open the fingers more easily.
    double innerForceScale = 1; // Scale factor for the force sensor output
    double outerForce = outerForceScale * (thumbOuterSensor->getForce() +
                                           indexOuterSensor->getForce()); // Sum the force from the outer sensors

    double innerForce = innerForceScale * (thumbInnerSensor->getForce() +
                                           indexInnerSensor->getForce()); // Sum the force from the inner sensors

    netFingerForce = (outerForce - innerForce);

    if (useAdmittanceControl) {
        double forceError = desiredFingerForce - netFingerForce; // Force error
        desiredFingerVel =
            IOAC.solve(forceError) * 1000 / COUNT_TO_MM; // Convert the meter/sec output from the conttoller to count/sec
    } else
        desiredFingerVel = generatedFingerWave;

    desiredFingerForce = netFingerForce > 0 ? 0 : desiredFingerForce;

    // Next 3 lines have an effect only if the controller mode is "POSITION_CONTROL", "VELOCITY_CONTROL",
    // "FORCE_CONTROL" respectively.
    finger->setDesiredPosition(generatedFingerWave); // Set the desired position with as the waveform output.
    finger->setDesiredVelocity(desiredFingerVel);    // Set the desired velocity as the output of the IOAC.
    finger->setDesiredForce(desiredFingerForce);     // Set the desired force

    finger->setForce(netFingerForce);

    if (finger->isHoming() || finger->isSharedLimitSwitchBusy()) {
        holdingRigidBody = false;
    } else if (!(fingerButtonState == FunctionalButton::SingleHold || fingerButtonState == FunctionalButton::DoubleHold)) {
        if (finger->getControllerMode() != MotorWithEncoder::FORCE_CONTROL) {
            holdingRigidBody = false;
            steadyState = finger->go(); // Calculate the internal controller output and actuate the motor
            if (steadyState)
                finger->release();
        } else { // Implement rigid body using position control in force control mode.
            float proximityThreshold = 200;
            // If the finger is close to the object, brake the motor before moving to position control mode.
            // This prevents the motor jittering when the finger is touching (or too close) the object.
            if (finger->getPos() < (FINGER_UPPER_SOFT_LIMIT - obj.nominalWidth) &&
                finger->getPos() >= (FINGER_UPPER_SOFT_LIMIT - (obj.nominalWidth + proximityThreshold)) &&
                obj.stiffness >= RIGID_BODY_STIFFNESS_THRESHOLD && netFingerForce < 0) {
                finger->brake();
                // A hold that starts after the brake must not use the errors of the previous hold. holdingRigidBody
                // stays true, so the force controller is still reset when the finger leaves the object.
                finger->resetHold();
            } else if (finger->getPos() >= (FINGER_UPPER_SOFT_LIMIT - obj.nominalWidth) &&
                       obj.stiffness >= RIGID_BODY_STIFFNESS_THRESHOLD && netFingerForce < 0) {
                // If trying to push a stiff object, hold the finger at the object. holdPosition() has its own
                // controller, so the force controller keeps its state.
                if (!holdingRigidBody)
                    finger->resetHold();
                finger->holdPosition(FINGER_UPPER_SOFT_LIMIT - obj.nominalWidth);
                holdingRigidBody = true;
            } else {
                // The force controller did not run during the hold, so its state is out of date
                if (holdingRigidBody)
                    finger->resetController();
                holdingRigidBody = false;
                steadyState = finger->go(); // Calculate the internal controller output and actuate the motor
                if (steadyState)
                    finger->release();
            }
        }
    } else if (fingerButtonState == FunctionalButton::SingleHold && !palmButton->read())
        finger->rotate(1, 0.22);
    else if (fingerButtonState == FunctionalButton::DoubleHold && !palmButton->read())
        finger->rotate(-1, 0.22);

    obj.inContactPrev = obj.inContact;
}
void HapticDevice::palmControlLoop() {
    palm->setDesiredVelocity(generatedPalmWave);
    palm->setDesiredPosition(generatedPalmWave);
    if (palm->isHoming() || palm->isSharedLimitSwitchBusy()) {
        return;
    } else if (!(palmButtonState == FunctionalButton::SingleHold || palmButtonState == FunctionalButton::DoubleHold)) {
        palm->go();
    } else if (palmButtonState == FunctionalButton::SingleHold && !fingerButton->read()) // Move up
        palm->forceRotate(1, .5);
    // Double holding the palm button homes the palm. update() starts it, because home() must not be called from this
    // interrupt.
    // palm->forceRotate(1, 1);
}

// Called when the host stops sending messages. Releases the motors and clears the commands of the host, so that the
// device does not continue a waveform or a virtual object without a host watching it.
void HapticDevice::stopForHostTimeout() {
    noInterrupts();  // The control loop interrupt uses these
    hostTimedOut = true;
    finger->softStop = true;
    palm->softStop = true;
    obj = PseudoObject();
    interrupts();

    WaveformGenerator *generators[] = {&waveformGeneratorFinger, &waveformGeneratorPalm};
    for (WaveformGenerator *generator : generators) {
        generator->setType(WaveformGenerator::WaveType::CONSTANT);
        generator->setFrequency(0);
        generator->setAmplitude(0);
        generator->setOffset(0);
    }
}

void HapticDevice::resumeAfterHostTimeout() {
    noInterrupts();
    hostTimedOut = false;
    finger->softStop = false;
    palm->softStop = false;
    interrupts();
}

void HapticDevice::resetFingerEncoder() { finger->resetEncoderPos(); }

void HapticDevice::resetPalmEncoder() { palm->resetEncoderPos(); }

void HapticDevice::setSerial(Stream &serial) { this->serial = &serial; }
