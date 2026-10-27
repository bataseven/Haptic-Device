// Minimal stand-in for the Arduino core, so the libraries can be compiled and tested on a computer.
// Time and pins are simulated: tests set the time with fake::setMicros() and the analog inputs with
// fake::analogInputs(), and read the PWM outputs with fake::analogOutputs().
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <map>

#define PI 3.1415926535897932384626433832795
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define LED_BUILTIN 13

typedef bool boolean;
typedef uint8_t byte;

namespace fake {
// micros() is 32 bits on the Teensy, so it is kept in 32 bits here too.
inline uint32_t &microsValue() {
    static uint32_t value = 0;
    return value;
}
inline void setMicros(uint32_t value) { microsValue() = value; }
inline void advanceMicros(uint32_t value) { microsValue() += value; }

inline std::map<uint8_t, int> &analogOutputs() {
    static std::map<uint8_t, int> values;
    return values;
}
inline std::map<uint8_t, int> &analogInputs() {
    static std::map<uint8_t, int> values;
    return values;
}
inline std::map<uint8_t, int> &digitalInputs() {
    static std::map<uint8_t, int> values;
    return values;
}
}  // namespace fake

inline unsigned long micros() { return fake::microsValue(); }
inline unsigned long millis() { return fake::microsValue() / 1000; }
inline void delay(unsigned long ms) { fake::advanceMicros(ms * 1000); }

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t pin) { return fake::digitalInputs()[pin]; }
inline void analogWrite(uint8_t pin, int value) { fake::analogOutputs()[pin] = value; }
inline int analogRead(uint8_t pin) { return fake::analogInputs()[pin]; }
inline void analogWriteResolution(int) {}
inline void analogReadResolution(int) {}

inline void noInterrupts() {}
inline void interrupts() {}

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

template <class A, class B>
auto min(A a, B b) -> decltype(a < b ? a : b) {
    return a < b ? a : b;
}

template <class A, class B>
auto max(A a, B b) -> decltype(a > b ? a : b) {
    return a > b ? a : b;
}

// Same as map() of the Teensy core
template <class T, class A, class B, class C, class D>
long map(T x, A inMin, B inMax, C outMin, D outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
