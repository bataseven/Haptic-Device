#include "FunctionalButton.h"

#include "Arduino.h"

FunctionalButton::FunctionalButton(uint8_t pin) {
    pinMode(pin, INPUT_PULLUP);
    _pin = pin;
}

// Reads the button input and updates the state of the button
// This function needs to be called as often as possible
// To get the state of the button, use getState()
void FunctionalButton::update() {
    _state = _update();
    _buttonReadingPrev = this->read();
}

u_int8_t FunctionalButton::_update() {
    _msec = millis();
    _buttonReading = digitalRead(_pin);
    if (_buttonReading && !_buttonReadingPrev) {
        _msecLst2 = millis();
    }
    if (_buttonReading == HIGH && _msec - _msecLst > _holdTimeout && _doubleHold) {
        _msecLst = 1;
        _prevReturned = DoubleHold;
        return DoubleHold;
    } else if (_msecLst && (_msec - _msecLst) > _clickTimeout && _buttonReading == LOW) {
        _msecLst = 0;
        _doubleHold = false;
        if (_prevReturned == DoubleHold)
            return None;
        else
            return SingleClick;
    } else if (!_msecLst && _msec - _msecLst2 > _holdTimeout && _buttonReading == HIGH) {
        _msecLst = 0;
        _prevReturned = SingleHold;
        return SingleHold;
    }

    if (_buttonReadingPrev != _buttonReading) {
        if (millis() - _debStartTime < _debDuration) {
            _prevReturned = None;
            return None;
        }
        _debStartTime = millis();

        _buttonReadingPrev = _buttonReading;

        if (_buttonReading == LOW) {  // press
            if (_msecLst) {           // 2nd press
                _msecLst = 0;
                _doubleHold = false;
                _prevReturned = DoubleClick;
                return DoubleClick;
            } else if (_prevReturned != SingleHold) {
                _doubleHold = true;
                _msecLst = 0 == _msec ? 1 : _msec;
                _prevReturned = None;
                return None;  // SINGLE?
            }
        }
    }
    _prevReturned = None;
    return None;
}

// Returns the state of the button
// The state is defined as follows:
// 0 - None: The finger button is not pressed.
// 1 - SingleClick: The button is pressed once.
// 2 - SingleHold: The button is pressed and held.
// 3 - DoubleClick: The button is pressed twice in a short time.
// 4 - DoubleHold: The button is pressed twice and held.
uint8_t FunctionalButton::getState() { return _state; }

uint8_t FunctionalButton::read() { return digitalRead(_pin); }

void FunctionalButton::setHoldTimeout(u_int32_t holdTimeout) { _holdTimeout = holdTimeout; }

void FunctionalButton::setCickTimeout(u_int32_t clickTimeout) { _clickTimeout = clickTimeout; }

void FunctionalButton::setDebounceDuration(u_int32_t debDuration) { _debDuration = debDuration; }

u_int32_t FunctionalButton::getHoldTimeout() { return _holdTimeout; }

u_int32_t FunctionalButton::getClickTimeout() { return _clickTimeout; }

u_int32_t FunctionalButton::getDebounceDuration() { return _debDuration; }

u_int8_t FunctionalButton::getPin() { return _pin; }
