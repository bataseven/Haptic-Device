// Stand-in for the Encoder library. Tests move the motor by changing the position of its encoder.
#pragma once

#include <stdint.h>

#include <vector>

class Encoder;

namespace fake {
// Every encoder that was created, so tests can reach the encoder of a motor.
inline std::vector<Encoder *> &encoders() {
    static std::vector<Encoder *> list;
    return list;
}
}  // namespace fake

class Encoder {
   public:
    Encoder(uint8_t, uint8_t) { fake::encoders().push_back(this); }
    int32_t read() { return position; }
    int32_t readAndReset() {
        int32_t value = position;
        position = 0;
        return value;
    }
    void write(int32_t value) { position = value; }

    int32_t position = 0;
};
