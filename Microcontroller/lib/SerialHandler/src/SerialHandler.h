/*"""

 Serial Message Format:
 First character of the message is the message type for both sending and reception.
 The message type is followed by the message payload.
 
   - While sending to haptic device:
        "G" + <Gripper Width> + SERIAL_SEPERATOR_CH + <Gripper Speed> + SERIAL_SEPERATOR_CH + <Gripper Force>
        "C" + <Working Modes or Commands>

    - While receiving from haptic device:
        "T" + <Next Point On The Trajectory>
        
"""*/
#ifndef SerialHandler_H
#define SerialHandler_H
#include "advancedSerial.h"
#include "HapticDevice.h"
#include <inttypes.h>

enum SerialConnectionApp
{
    UNITY = 0,
    HAPTIC_SERIAL_INTERFACE = 1,
};

class SerialHandler : public advancedSerial {
    public:
    void update();
    void setSerial(Stream &serial);
    void setStartMarker(char startMarker);
    void setEndMarker(char endMarker);
    void setSeperator(char seperator);
    void setDebug(bool debug);
    void setPrintFrequency(float printFrequency);
    void parseString(char *string);
    void printGains();
    void printLimits();
    void setHapticDevice(HapticDevice* hapticDevice);
    Stream & getSerial();

    private:
    SerialConnectionApp _app = UNITY;
    float _printFrequency = 50;
    bool _debug = false;
    bool _serialOpen = false;
    bool _serialOpenPrev = false;
    long _periodicTimer = 0;
    HapticDevice* device;
    Stream *_serial;
    char _startMarker = '<';
    char _endMarker = '>';
    char _separator = '#';
    static const byte _numChars = 128;
    char _receivedChars[_numChars];

    void _printPeriodically(float frequency, bool debug);
    void _receiveNonBlocking(void);
    
};

#include "Arduino.h"
#endif