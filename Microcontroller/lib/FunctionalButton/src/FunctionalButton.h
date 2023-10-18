#ifndef FunctionalButton_H
#define FunctionalButton_H
#include "Arduino.h"

class FunctionalButton {
   public:
    enum { None, SingleClick, SingleHold, DoubleClick, DoubleHold };
    FunctionalButton(uint8_t);
    void update();
    uint8_t getState();
    uint8_t read();

    void setHoldTimeout(u_int32_t);
    void setCickTimeout(u_int32_t);
    void setDebounceDuration(u_int32_t);

    u_int32_t getHoldTimeout();
    u_int32_t getClickTimeout();
    u_int32_t getDebounceDuration();
    u_int8_t getPin();

   private:
    uint8_t _pin;
    uint8_t _state = None;
    byte _buttonReading;
    byte _buttonReadingPrev;
    u_int32_t _clickTimeout = 300;
    u_int32_t _holdTimeout = 1000;
    u_int32_t _msec = millis();
    u_int32_t _msecLst = 0;
    u_int32_t _msecLst2 = 0;
    byte _doubleHold = false;
    u_int32_t _debDuration = 40;
    u_int32_t _debStartTime = 0;
    u_int8_t _prevReturned = 0;

    u_int8_t _update();
};
#endif