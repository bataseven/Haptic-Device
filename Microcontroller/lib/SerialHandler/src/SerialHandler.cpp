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

To set desired force:   "D" + <Desired Force> => "D11.5"

To send the stiffness and the nominal width of the object: "P" + <Nominal Width> + # + <Stiffness> + # + <Damping> =>
"P50000#2.5#0.1"

To set the limits (hard or soft) for the motors: "L" + "H" or "S" + <Motor Index> + <Lower Limit> + # + <Upper Limit> =>
"LS01000#2000"

To home the motors: "H" + <Motor Index> => "H1"

To toggle set the serial handler debug mode: "B" + <Debug Mode> => "B1" or "B0"


  - While receiving from haptic device:

        (No longer used) Desired aperture of the gripper:    "T" + <Next Point On The Trajectory>

        PID gains of every controller mode: "K" + <Motor Index> + <Kp1> + # + <Ki1> + # + <Kd1> + # + <Kp2> + # + <Ki2>
+ # + <Kd2> + # + <Kp3> + # + <Ki3> + # + <Kd3>


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

MPU6050 Euler Angles:               "E" + <Yaw> + SERIAL_SEPERATOR_CH + <Roll> + SERIAL_SEPERATOR_CH + <Pitch> =>
"E10.2#10.2#10.2" MPU6050 Quaternions: "Q" + <Q0> + SERIAL_SEPERATOR_CH + <Q1> + SERIAL_SEPERATOR_CH + <Q2> +
SERIAL_SEPERATOR_CH + <Q3> => "Q0.5#0.5#0.5#0.5"
"""*/
void SerialHandler::update() {
    _printPeriodically(_printFrequency, _debug);
    _receiveNonBlocking();
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

    if (_serial->available() <= 0) return;
    
    rc = _serial->read();
    if (recvInProgress == true) {
        if (rc != _endMarker) {
            _receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= _numChars) {
                this->pln("Buffer Overflow");
                ndx = _numChars - 1;
            }
        } else {
            // digitalWrite(13, !digitalRead(13));
            _receivedChars[ndx] = '\0';  // terminate the string when the end marker arrives
            recvInProgress = false;
            ndx = 0;
            this->parseString(_receivedChars);
        }
    }

    else if (rc == _startMarker) {
        recvInProgress = true;
    }
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
                char *token = strtok(string, seperator);
                device->wsg50.width = atof(token);
                token = strtok(NULL, seperator);
                device->wsg50.speed = atof(token);
                token = strtok(NULL, seperator);
                device->wsg50.force = atof(token);
            }
            break;

        case 'C':
            if (strlen(string) > 0) {
                _app = (SerialConnectionApp)atoi(string);
            }
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
                if (motorIndex == '0')
                    device->finger->home();
                else if (motorIndex == '1')
                    device->palm->home();
            }
            break;

        case 'K':
        case 'k':
            if (strlen(string) > 0) {
                char gainType = string[0];
                char motorIndex = string[1];
                char *token = strtok(string, seperator);  // Remove the gain type and motor index
                token = strtok(NULL, seperator);          // Token now contains the gain value
                double gain = atof(token);
                if (gain < 0) break;  // Do not accept negative values
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
            }
            this->printGains();
            break;

        case 'W':
        case 'w':
            if (strlen(string) > 0) {
                char valueType = string[0];

                char generatorIndex = string[1];

                char *token = strtok(string, seperator);  // Remove the generator index

                token = strtok(NULL, seperator);  // Token now contains the value

                double value = atof(token);

                if (valueType == 'T' || valueType == 't') {
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
                char modeString = string[1];
                int mode = atoi(&modeString);
                if (motorIndex == '0')
                    device->finger->setControllerMode(MotorWithEncoder::ControllerMode(mode));
                else if (motorIndex == '1')
                    device->palm->setControllerMode(MotorWithEncoder::ControllerMode(mode));
                // this->p("Palm Controller mode: ").pln(device->palm->getControllerMode());
                // this->p("Finger Controller mode: ").pln(device->finger->getControllerMode());
            }
            this->printGains();
            this->printLimits();
            break;

        case 'd':
        case 'D':
            if (strlen(string) > 0) {
                device->desiredFingerForce = atof(string);
            }
            break;

        case 'P':
        case 'p':
            if (strlen(string) > 0) {
                char *token = strtok(string, seperator);
                device->obj.nominalWidth = atof(token);
                token = strtok(NULL, seperator);
                device->obj.stiffness = atof(token);
                token = strtok(NULL, seperator);
                device->obj.damping = atof(token);
            }
            break;

        case 'l':
        case 'L':
            if (strlen(string) > 0) {
                char limitType = string[0];
                char motorIndex = string[1];
                // remove the first two characters from the string
                memmove(string, string + 2, strlen(string) - 1);
                char *token = strtok(string, seperator);
                float lower = atof(token);
                token = strtok(NULL, seperator);
                float upper = atof(token);
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
    MotorWithEncoder::ControllerMode temp = device->finger->getControllerMode();
    device->finger->setControllerMode(MotorWithEncoder::POSITION_CONTROL);
    double p1 = device->finger->getKp();
    double i1 = device->finger->getKi();
    double d1 = device->finger->getKd();
    device->finger->setControllerMode(MotorWithEncoder::VELOCITY_CONTROL);
    double p2 = device->finger->getKp();
    double i2 = device->finger->getKi();
    double d2 = device->finger->getKd();
    device->finger->setControllerMode(MotorWithEncoder::FORCE_CONTROL);
    double p3 = device->finger->getKp();
    double i3 = device->finger->getKi();
    double d3 = device->finger->getKd();
    device->finger->setControllerMode(temp);
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

    temp = device->palm->getControllerMode();
    device->palm->setControllerMode(MotorWithEncoder::POSITION_CONTROL);
    p1 = device->palm->getKp();
    i1 = device->palm->getKi();
    d1 = device->palm->getKd();
    device->palm->setControllerMode(MotorWithEncoder::VELOCITY_CONTROL);
    p2 = device->palm->getKp();
    i2 = device->palm->getKi();
    d2 = device->palm->getKd();
    device->palm->setControllerMode(MotorWithEncoder::FORCE_CONTROL);
    p3 = device->palm->getKp();
    i3 = device->palm->getKi();
    d3 = device->palm->getKd();
    device->palm->setControllerMode(temp);
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

void SerialHandler::setPrintFrequency(float freq) { _printFrequency = freq; }
void SerialHandler::setDebug(bool debug) { _debug = debug; }
void SerialHandler::setHapticDevice(HapticDevice *hapticDevice) { this->device = hapticDevice; }

Stream &SerialHandler::getSerial() {
    Stream *serial = &*_serial;
    return *serial;
}