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

    void setHoldTimeout(uint32_t);
    void setClickTimeout(uint32_t);
    void setDebounceDuration(uint32_t);

    uint32_t getHoldTimeout();
    uint32_t getClickTimeout();
    uint32_t getDebounceDuration();
    uint8_t getPin();

   private:
    uint8_t _pin;
    uint8_t _state = None;
    byte _buttonReading;
    byte _buttonReadingPrev;
    uint32_t _clickTimeout = 300;
    uint32_t _holdTimeout = 1000;
    uint32_t _msec = millis();
    uint32_t _msecLst = 0;
    uint32_t _msecLst2 = 0;
    byte _doubleHold = false;
    uint32_t _debDuration = 40;
    uint32_t _debStartTime = 0;
    uint8_t _prevReturned = 0;

    uint8_t _update();
};
#endif