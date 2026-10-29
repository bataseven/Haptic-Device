#include "SerialHandler.h"

/*"""

 Serial Message Format:
 Every message sent and received over the serial should start with a '_startMarker' and end with an '_endMarker'.
 First character (after the start character) of the message is the message type for both sending and reception.
 The message type is followed by the message payload.
 Different values inside the payload are separated by a '_separator'.
 SERIAL_SEPERATOR_CH = '#'
_separator = '#'

  - While sending to haptic device:
        To get gripper data:    "G" + <Gripper Width> + SERIAL_SEPERATOR_CH + <Gripper Speed> + SERIAL_SEPERATOR_CH +
<Gripper Force>  => "G0.5#0.5#0.5"

To specify which program is connected to the haptic device over the serial port: "C" + <Program> => "C0"
List of programs are defined as enum in the header file.

To stop the motors:     "S" + <Motor Index> + <Stop(0 or 1)> => "S01"

To change PID gains:    "K" + "P" or "I" or "D" + <MotorIndex> + SERIAL_SEPERATOR_CH + <Gain> => "KP1#0.5"

To change waveforms:    "W" + "T" or "F" or "A" or "O" + <Generator Index> + SERIAL_SEPERATOR_CH + <Value> => "WF1#0.5"

To change the controller mode: "O" + <Motor Index> + <ControllerMode> => "O10"

To send the stiffness and the nominal width of the object: "P" + <Nominal Width> + # + <Stiffness> + # + <Damping> =>
"P50000#2.5#0.1"

To set the limits (hard or soft) for the motors: "L" + "H" or "S" + <Motor Index> + <Lower Limit> + # + <Upper Limit> =>
"LS01000#2000"

To home the motors: "H" + <Motor Index> => "H1"

To toggle set the serial handler debug mode: "B" + <Debug Mode> => "B1" or "B0"

To arm the host timeout: "T" + <Timeout in ms> => "T1000". "T0" disarms it. While armed, if no message is received for
the timeout, the motors are released, the waveforms are set to constant 0, the virtual object is removed and the timeout
is disarmed. The motors stay released until the next message of any kind. It is not armed by default, so hosts that
never send "T" are not affected.

Heartbeat: "A". Does nothing, but like any message it resets the host timeout.


  - While receiving from haptic device:

        (No longer used) Desired aperture of the gripper:    "T" + <Next Point On The Trajectory>

        PID gains of every controller mode: "K" + <Motor Index> + <Kp1> + # + <Ki1> + # + <Kd1> + # + <Kp2> + # + <Ki2>
+ # + <Kd2> + # + <Kp3> + # + <Ki3> + # + <Kd3>


Device time: "R" + <micros()> => "R123456789". First message of each status line sent to the haptic serial interface.
The time when the values of the line were read, in microseconds. It overflows every 71.6 minutes.

Haptic device status: "H" + <Motor Index> + <Encoder Pos>
+ SERIAL_SEPERATOR_CH + <Encoder Speed> => "H1123#123" (123 pos 123 speed)

Waveform values: "W" + <Generator Index> + <Value> => "W110.5" (10.5 value of generator 1)

Force Values:                       "F" +
<Force1> + SERIAL_SEPERATOR_CH + <Force2> + SERIAL_SEPERATOR_CH + <Force3> + SERIAL_SEPERATOR_CH + <Force4> =>
"F0.5#0.5#0.5#0.5"

Net Finger Force:                   "N" + <Net Finger Force>

Desired Finger Force:               "D" + <Desired Finger Force>

Limits (hard or soft) for the motors: "L" + "H" or "S" + <Motor Index> + <Lower Limit> + # + <Upper Limit> =>
"LS01000#2000"

Host timeout: "X" + <Timeout in ms>. Sent when the motors are stopped because of the host timeout.

Controller mode: "M" + <Motor Index> + <Controller Mode> => "M02" (finger in force control). Sent at startup, after
the controller mode changes, and with the gains and limits when "C1" is received.

Encoder count to mm factors: "U" + <Finger> + # + <Palm> => "U0.000296452618#0.005973945762". Sent when "C1" is
received.

MPU6050 Euler Angles:               "E" + <Yaw> + SERIAL_SEPERATOR_CH + <Roll> + SERIAL_SEPERATOR_CH + <Pitch> =>
"E10.2#10.2#10.2" MPU6050 Quaternions: "Q" + <Q0> + SERIAL_SEPERATOR_CH + <Q1> + SERIAL_SEPERATOR_CH + <Q2> +
SERIAL_SEPERATOR_CH + <Q3> => "Q0.5#0.5#0.5#0.5"
"""*/
void SerialHandler::update() {
    _printPeriodically(_printFrequency, _debug);
    _receiveNonBlocking();
    _checkHostTimeout();
    _reportNewHomingFailures();
    // Activate interrupts if the serial port is closed
    // if(!*this->_serial) {
    //     digitalWrite(13, HIGH);
    // }
    // else {
    //     digitalWrite(13, LOW);
    // }
}

void SerialHandler::setSerial(Stream &serial) {
    _serial = &serial;
    advancedSerial::setPrinter(serial);
}

void SerialHandler::setEndMarker(char endMarker) { _endMarker = endMarker; }

void SerialHandler::setStartMarker(char startMarker) { _startMarker = startMarker; }

void SerialHandler::setSeperator(char seperator) { _separator = seperator; }

void SerialHandler::_receiveNonBlocking() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;

    while (_serial->available() > 0) {
        rc = _serial->read();
        if (recvInProgress == true) {
            if (rc != _endMarker) {
                _receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= _numChars) {
                    // Drop the message instead of parsing the cut off part. Wait for the next start marker.
                    this->pln("Buffer Overflow");
                    recvInProgress = false;
                    ndx = 0;
                }
            } else {
                _receivedChars[ndx] = '\0';  // terminate the string when the end marker arrives
                recvInProgress = false;
                ndx = 0;
                _lastMessageTime = millis();
                if (device->hostTimedOut) device->resumeAfterHostTimeout();  // A host is sending messages again
                this->parseString(_receivedChars);
            }
        }

        else if (rc == _startMarker) {
            recvInProgress = true;
        }
    }
}

// Reads the next token of strtok() as a number. Pass the string for the first token and NULL for the next ones.
// Returns false if the token is missing or is not a finite number, so a malformed message can be ignored.
static bool nextNumber(char *string, const char *separator, double &value) {
    char *token = strtok(string, separator);
    if (token == NULL) return false;
    value = atof(token);
    return isfinite(value);
}

void SerialHandler::parseString(char *string) {
    const char seperator[2] = {_separator, '\0'};
    char messageType = string[0];
    // Remove the first character from the string using memmove. First character is the message type.
    memmove(string, string + 1, strlen(string));

    switch (messageType) {
        // Gripper state messages start with G
        case 'G':
            if (strlen(string) > 0) {
                double width, speed, force;
                if (!nextNumber(string, seperator, width) || !nextNumber(NULL, seperator, speed) ||
                    !nextNumber(NULL, seperator, force))
                    break;
                device->wsg50.width = width;
                device->wsg50.speed = speed;
                device->wsg50.force = force;
            }
            break;

        case 'C':
            if (strlen(string) > 0) {
                int app = atoi(string);
                if (app == UNITY || app == HAPTIC_SERIAL_INTERFACE) _app = (SerialConnectionApp)app;
                if (_app == HAPTIC_SERIAL_INTERFACE) {
                    // Send the current state, so the interface shows it even if the device was not reset
                    this->printGains();
                    this->printLimits();
                    this->printModes();
                    this->printUnits();
                    this->printHomingFailures();
                }
            }
            break;

        case 'T':
        case 't':
            // Arms the host timeout with the given milliseconds, 0 disarms it.
            if (strlen(string) > 0) {
                long timeout = atol(string);
                if (timeout >= 0) _hostTimeout = timeout;
            }
            break;

        case 'A':
        case 'a':
            // Heartbeat. Receiving any message resets the host timeout, so there is nothing else to do.
            break;

        case 'S':
        case 's':
            if (strlen(string) > 0) {
                char motorIndex = string[0];
                char stop = string[1];
                if (motorIndex == '0')
                    device->finger->hardStop = stop == '1';
                else if (motorIndex == '1')
                    device->palm->hardStop = stop == '1';
            }
            break;

        case 'H':
        case 'h':
            if (strlen(string) > 0) {
                char motorIndex = string[0];
                noInterrupts();  // home() also changes the other motor's state, which the control loop interrupt uses
                if (motorIndex == '0')
                    device->finger->home();
                else if (motorIndex == '1')
                    device->palm->home();
                interrupts();
            }
            break;

        case 'K':
        case 'k':
            if (strlen(string) > 0) {
                char gainType = string[0];
                char motorIndex = string[1];
                strtok(string, seperator);  // Remove the gain type and motor index
                double gain;
                if (!nextNumber(NULL, seperator, gain)) break;
                if (gain < 0) break;  // Do not accept negative values
                noInterrupts();       // The control loop interrupt uses the gains
                if (gainType == 'P' || gainType == 'p') {
                    if (motorIndex == '0') {
                        device->finger->setKp(gain);
                        // this->p("Finger Kp: ").pln(gain);
                    } else if (motorIndex == '1') {
                        device->palm->setKp(gain);
                        // this->p("Palm Kp: ").pln(gain);
                    }
                } else if (gainType == 'I' || gainType == 'i') {
                    if (motorIndex == '0') {
                        device->finger->setKi(gain);
                        // this->p("Finger Ki: ").pln(gain);
                    } else if (motorIndex == '1') {
                        device->palm->setKi(gain);
                        // this->p("Palm Ki: ").pln(gain);
                    }
                } else if (gainType == 'D' || gainType == 'd') {
                    if (motorIndex == '0') {
                        device->finger->setKd(gain);
                        // this->p("Finger Kd: ").pln(gain);
                    } else if (motorIndex == '1') {
                        device->palm->setKd(gain);
                        // this->p("Palm Kd: ").pln(gain);
                    }
                }
                interrupts();
            }
            this->printGains();
            break;

        case 'W':
        case 'w':
            if (strlen(string) > 0) {
                char valueType = string[0];

                char generatorIndex = string[1];

                strtok(string, seperator);  // Remove the value type and the generator index

                double value;
                if (!nextNumber(NULL, seperator, value)) break;

                if (valueType == 'T' || valueType == 't') {
                    if (value < WaveformGenerator::CONSTANT || value > WaveformGenerator::TRAPEZOIDAL) break;
                    if (generatorIndex == '0') {
                        device->waveformGeneratorFinger.setType((WaveformGenerator::WaveType((int)value)));
                        // this->p("Finger waveform type: ").pln(value);
                    } else if (generatorIndex == '1') {
                        device->waveformGeneratorPalm.setType((WaveformGenerator::WaveType((int)value)));
                        // this->p("Palm waveform type: ").pln(value);
                    }
                }
                if (valueType == 'F' || valueType == 'f') {
                    if (generatorIndex == '0') {
                        device->waveformGeneratorFinger.setFrequency(value);
                        // this->p("Finger waveform frequency: ").pln(value);
                    } else if (generatorIndex == '1') {
                        device->waveformGeneratorPalm.setFrequency(value);
                        // this->p("Palm waveform frequency: ").pln(value);
                    }
                }
                if (valueType == 'A' || valueType == 'a') {
                    if (generatorIndex == '0') {
                        device->waveformGeneratorFinger.setAmplitude(value);
                        // this->p("Finger waveform amplitude: ").pln(value);
                    } else if (generatorIndex == '1') {
                        // ! TODO: The implications of the next line have not been tested to the fullest extent.
                        // TODO: I made it for the palm expriments.
                        if (device->waveformGeneratorPalm.getAmplitude() != value) device->palm->resetIntegralTerm();
                        device->waveformGeneratorPalm.setAmplitude(value);
                        // this->p("Palm waveform amplitude: ").pln(value);
                    }
                }
                if (valueType == 'O' || valueType == 'o') {
                    if (generatorIndex == '0') {
                        device->waveformGeneratorFinger.setOffset(value);
                        // this->p("Finger waveform offset: ").pln(value);
                    } else if (generatorIndex == '1') {
                        device->waveformGeneratorPalm.setOffset(value);
                        // this->p("Palm waveform offset: ").pln(value);
                    }
                }
            }
            break;

        case 'O':
        case 'o':
            if (strlen(string) > 0) {
                char motorIndex = string[0];
                int mode = string[1] - '0';
                if (mode < MotorWithEncoder::POSITION_CONTROL || mode > MotorWithEncoder::FORCE_CONTROL) break;
                noInterrupts();  // The control loop interrupt uses the controller mode
                if (motorIndex == '0')
                    device->finger->setControllerMode(MotorWithEncoder::ControllerMode(mode));
                else if (motorIndex == '1')
                    device->palm->setControllerMode(MotorWithEncoder::ControllerMode(mode));
                interrupts();
                // this->p("Palm Controller mode: ").pln(device->palm->getControllerMode());
                // this->p("Finger Controller mode: ").pln(device->finger->getControllerMode());
            }
            this->printGains();
            this->printLimits();
            this->printModes();
            break;

        case 'P':
        case 'p':
            if (strlen(string) > 0) {
                double width, stiffness, damping;
                if (!nextNumber(string, seperator, width) || !nextNumber(NULL, seperator, stiffness) ||
                    !nextNumber(NULL, seperator, damping))
                    break;
                noInterrupts();  // Change the object at once, the control loop interrupt uses it
                device->obj.nominalWidth = width;
                device->obj.stiffness = stiffness;
                device->obj.damping = damping;
                interrupts();
            }
            break;

        case 'l':
        case 'L':
            if (strlen(string) >= 2) {
                char limitType = string[0];
                char motorIndex = string[1];
                // remove the first two characters from the string
                memmove(string, string + 2, strlen(string) - 1);
                double lower, upper;
                if (!nextNumber(string, seperator, lower) || !nextNumber(NULL, seperator, upper)) break;
                if (motorIndex == '0') {
                    if (limitType == 'S' || limitType == 's')
                        device->finger->setSoftPosLimit(lower, upper);
                    else if (limitType == 'H' || limitType == 'h')
                        device->finger->setHardPosLimit(lower, upper);

                } else if (motorIndex == '1') {
                    if (limitType == 'S' || limitType == 's')
                        device->palm->setSoftPosLimit(lower, upper);

                    else if (limitType == 'H' || limitType == 'h')
                        device->palm->setHardPosLimit(lower, upper);
                }
                this->printLimits();
            }
            break;

        case 'b':
        case 'B':
            if (strlen(string) > 0) {
                char debug = string[0];
                if (debug == '0') {
                    this->setDebug(false);
                    this->pln("<B0>");
                } else if (debug == '1') {
                    this->setDebug(true);
                    this->pln("<B1>");
                }
            }
            break;
    }
}

void SerialHandler::printGains() {
    // The gains are read per mode. Switching the controller mode to read them would affect the running control loop.
    double p1 = device->finger->getKp(MotorWithEncoder::POSITION_CONTROL);
    double i1 = device->finger->getKi(MotorWithEncoder::POSITION_CONTROL);
    double d1 = device->finger->getKd(MotorWithEncoder::POSITION_CONTROL);
    double p2 = device->finger->getKp(MotorWithEncoder::VELOCITY_CONTROL);
    double i2 = device->finger->getKi(MotorWithEncoder::VELOCITY_CONTROL);
    double d2 = device->finger->getKd(MotorWithEncoder::VELOCITY_CONTROL);
    double p3 = device->finger->getKp(MotorWithEncoder::FORCE_CONTROL);
    double i3 = device->finger->getKi(MotorWithEncoder::FORCE_CONTROL);
    double d3 = device->finger->getKd(MotorWithEncoder::FORCE_CONTROL);
    this->p(_startMarker)
        .p("K0")
        .p(p1)
        .p(_separator)
        .p(i1, 6)
        .p(_separator)
        .p(d1)
        .p(_separator)
        .p(p2)
        .p(_separator)
        .print(i2, 6)
        .p(_separator)
        .p(d2)
        .p(_separator)
        .p(p3)
        .p(_separator)
        .p(i3, 6)
        .p(_separator)
        .p(d3)
        .pln(_endMarker);

    p1 = device->palm->getKp(MotorWithEncoder::POSITION_CONTROL);
    i1 = device->palm->getKi(MotorWithEncoder::POSITION_CONTROL);
    d1 = device->palm->getKd(MotorWithEncoder::POSITION_CONTROL);
    p2 = device->palm->getKp(MotorWithEncoder::VELOCITY_CONTROL);
    i2 = device->palm->getKi(MotorWithEncoder::VELOCITY_CONTROL);
    d2 = device->palm->getKd(MotorWithEncoder::VELOCITY_CONTROL);
    p3 = device->palm->getKp(MotorWithEncoder::FORCE_CONTROL);
    i3 = device->palm->getKi(MotorWithEncoder::FORCE_CONTROL);
    d3 = device->palm->getKd(MotorWithEncoder::FORCE_CONTROL);
    this->p(_startMarker)
        .p("K1")
        .p(p1)
        .p(_separator)
        .p(i1, 6)
        .p(_separator)
        .p(d1)
        .p(_separator)
        .p(p2)
        .p(_separator)
        .p(i2, 6)
        .p(_separator)
        .p(d2)
        .p(_separator)
        .p(p3)
        .p(_separator)
        .p(i3, 6)
        .p(_separator)
        .p(d3)
        .pln(_endMarker);
}

void SerialHandler::printModes() {
    this->p(_startMarker).p("M0").p((int)device->finger->getControllerMode()).pln(_endMarker);
    this->p(_startMarker).p("M1").p((int)device->palm->getControllerMode()).pln(_endMarker);
}

// The interface converts encoder counts to mm with these, so they are only defined in the firmware.
void SerialHandler::printUnits() {
    this->p(_startMarker).p("U").p(COUNT_TO_MM, 12).p(_separator).p(COUNT_TO_MM_2, 12).pln(_endMarker);
}

// Sends "Z" for each motor whose last homing gave up, so the host knows it is not homed.
void SerialHandler::printHomingFailures() {
    if (device->finger->homingFailed()) this->p(_startMarker).p("Z0").pln(_endMarker);
    if (device->palm->homingFailed()) this->p(_startMarker).p("Z1").pln(_endMarker);
}

// Sends "Z" once when the homing of a motor gives up. A host that connects later gets it with the "C" message.
void SerialHandler::_reportNewHomingFailures() {
    bool fingerFailed = device->finger->homingFailed();
    bool palmFailed = device->palm->homingFailed();
    if (fingerFailed && !_fingerHomingFailedPrev) this->p(_startMarker).p("Z0").pln(_endMarker);
    if (palmFailed && !_palmHomingFailedPrev) this->p(_startMarker).p("Z1").pln(_endMarker);
    _fingerHomingFailedPrev = fingerFailed;
    _palmHomingFailedPrev = palmFailed;
}

void SerialHandler::printLimits() {
    this->p(_startMarker)
        .p("LS0")
        .p(device->finger->getLowerSoftPosLimit())
        .p(_separator)
        .p(device->finger->getUpperSoftPosLimit())
        .pln(_endMarker);

    this->p(_startMarker)
        .p("LH0")
        .p(device->finger->getLowerHardPosLimit())
        .p(_separator)
        .p(device->finger->getUpperHardPosLimit())
        .pln(_endMarker);

    this->p(_startMarker)
        .p("LS1")
        .p(device->palm->getLowerSoftPosLimit())
        .p(_separator)
        .p(device->palm->getUpperSoftPosLimit())
        .pln(_endMarker);

    this->p(_startMarker)
        .p("LH1")
        .p(device->palm->getLowerHardPosLimit())
        .p(_separator)
        .p(device->palm->getUpperHardPosLimit())
        .pln(_endMarker);
}

void SerialHandler::_printPeriodically(float freq, bool debug = false) {
    if (freq <= 0) return;
    // Guard close to when the next print should happen
    if (millis() - _periodicTimer < 1000.0 / freq) return;
    _periodicTimer = millis();

    if (debug) {
        // Interrupts are disabled while accessing the device data
        noInterrupts();
        // float TI = device->thumbInnerSensor->getForce();
        // float TO = device->thumbOuterSensor->getForce();
        // float II = device->indexInnerSensor->getForce();
        // float IO = device->indexOuterSensor->getForce();
        // float F = device->netFingerForce;
        // float DF = device->desiredFingerForce;
        float FP = device->finger->getPos();
        float FW = device->generatedFingerWave;
        float FV = device->finger->getVel();
        float PP = device->palm->getPos();
        float PW = device->generatedPalmWave;
        float PV = device->palm->getVel();
        interrupts();

        // this->p("TI=").p(TI).p("").p(" ");
        // this->p("TO=").p(TO).p("").p(" ");
        // this->p("II=").p(II).p("").p(" ");
        // this->p("IO=").p(IO).p("").p(" ");
        // this->p("User Force=").p(F).p("").p(" ");
        // this->p("Desired Force=").p(DF).p("").p(" ");
        // this->p("ACC=").p(ACC).p("").p(" ");
        this->p("FP=").p(FP).p(_separator);
        this->p("FV=").p(FV).p(_separator);
        this->p("FW=").p(FW).p(_separator);
        this->p("PP=").p(PP).p(_separator);
        this->p("PV=").p(PV).p(_endMarker);
        this->p("PW=").p(PW).p(_separator);
    } else {
        noInterrupts();  // Interrupts are disabled while accessing the device data
        unsigned long snapshotTime = micros();  // Sent along, so the receiver doesn't depend on transfer delays
        int32_t posFinger = device->finger->getPos();
        int32_t velFinger = device->finger->getVel();
        int32_t posPalm = device->palm->getPos();
        int32_t velPalm = device->palm->getVel();
        double netFingerForce = device->netFingerForce;
        double desiredFingerForce = device->desiredFingerForce;
        bool imuActive = device->imuActive;
        // signed short heading = device->eulerAngles.h / 16;
        // signed short roll = device->eulerAngles.r / 16;
        // signed short pitch = device->eulerAngles.p / 16;
        float fingerWave = device->generatedFingerWave;
        float palmWave = device->generatedPalmWave;
        interrupts();  // Interrupts are enabled again

        // Send different information depending on whats on the other end of the serial connection
        switch (_app) {
            case HAPTIC_SERIAL_INTERFACE: {
                this->p(_startMarker).p("R").p(snapshotTime).p(_endMarker);

                this->p(_startMarker)
                    .p("H0")
                    .p(posFinger < 0 ? "-" : "")
                    .p(abs(posFinger), HEX)
                    .p(_separator)
                    .p(velFinger < 0 ? "-" : "")
                    .p(abs(velFinger), HEX)
                    .p(_endMarker);

                this->p(_startMarker)
                    .p("H1")
                    .p(posPalm < 0 ? "-" : "")
                    .p(abs(posPalm), HEX)
                    .p(_separator)
                    .p(velPalm < 0 ? "-" : "")
                    .p(abs(velPalm), HEX)
                    .p(_endMarker);

                this->p(_startMarker).p("N").p(netFingerForce).p(_endMarker);

                if (imuActive) {
                }
                // this->p(_startMarker).p("E").p(heading).p(_separator).p(roll).p(_separator).p(pitch).p(_endMarker);

                this->p(_startMarker)
                    .p("F")
                    .p(device->thumbOuterSensor->getForce())
                    .p(_separator)
                    .p(device->thumbInnerSensor->getForce())
                    .p(_separator)
                    .p(device->indexInnerSensor->getForce())
                    .p(_separator)
                    .p(device->indexOuterSensor->getForce())
                    .p(_endMarker);

                this->p(_startMarker).p("W0").p(fingerWave).p(_endMarker);

                this->p(_startMarker).p("W1").p(palmWave).p(_endMarker);

            } break;
            case UNITY: {
                // TODO - Delete the next line if you want to print the velocity
                velFinger = 0;
                this->p(_startMarker)
                    .p("H0")
                    .p(posFinger < 0 ? "-" : "")
                    .p(abs(posFinger), HEX)
                    .p(_separator)
                    .p(velFinger < 0 ? "-" : "")
                    .p(abs(velFinger), HEX)
                    .p(_endMarker);

                velPalm = 0;
                this->p("<H1")
                    .p(posPalm < 0 ? "-" : "")
                    .p(abs(posPalm), HEX)
                    .p(_separator)
                    .p(velPalm < 0 ? "-" : "")
                    .p(abs(velPalm), HEX)
                    .p(_endMarker);

                this->p(_startMarker).p("N").p(netFingerForce).p(_endMarker);
                this->p(_startMarker).p("D").p(desiredFingerForce).p(_endMarker);

                // Euler Angles
                if (imuActive) {
                }
                // this->p(_startMarker).p("E").p(heading).p(_separator).p(roll).p(_separator).p(pitch).p(_endMarker);
            } break;
            default:
                break;
        }
    }
    this->pln();
}

// Stops the device if the host armed the timeout (see the "T" message) and has not sent any message since.
void SerialHandler::_checkHostTimeout() {
    if (_hostTimeout == 0 || device->hostTimedOut) return;
    if (millis() - _lastMessageTime <= _hostTimeout) return;
    device->stopForHostTimeout();
    this->p(_startMarker).p("X").p(_hostTimeout).pln(_endMarker);
    // The host that armed the timeout is gone, so the next host should not be stopped unless it arms it too
    _hostTimeout = 0;
}

void SerialHandler::setPrintFrequency(float freq) { _printFrequency = freq; }
void SerialHandler::setDebug(bool debug) { _debug = debug; }
void SerialHandler::setHapticDevice(HapticDevice *hapticDevice) { this->device = hapticDevice; }

Stream &SerialHandler::getSerial() {
    Stream *serial = &*_serial;
    return *serial;
}