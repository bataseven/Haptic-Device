/********************************************************************
 
  - WaveformGenerator.cpp
  - A non-blocking waveform generator library.
  - Uses micros() to advance the phase of the waveform, so changing the frequency does not make the output jump.
  - Frequency, amplitude, offset manipulation.
  - Following waveforms available:
    -- CONSTANT,  
    -- SQUARE,    
    -- SAWTOOTH,  
    -- TRIANGULAR,
    -- SINUSOIDAL,
    -- TRAPEZOIDAL
    
  - Created by Berke Ataseven, 2021.

*********************************************************************/

#include "WaveformGenerator.h"

#include "Arduino.h"

WaveformGenerator::WaveformGenerator(){};                             // Constructor with default values.
WaveformGenerator::WaveformGenerator(WaveType type, float frequency,  // Constructor with the given parameters.
                                     float amplitude)
    : _type(type),
      _frequency(frequency),
      _period(1.0 / frequency),
      _amplitude(amplitude) {}

// Generate a point on the wave with the given parameters.
// This method should be called as frequently as possible to generate a smoother waveform.
float WaveformGenerator::generate() {
    // Advance the phase by the time elapsed since the last call. The waves are calculated from the phase instead of
    // the time since boot, so that the output does not jump when the frequency changes.
    uint32_t now = micros();
    if (_started) {
        uint32_t elapsed = now - _lastMicros;  // Unsigned subtraction also works when micros() overflows
        double dt = elapsed / 1000000.0;
        _phase = fmod(_phase + 2 * PI * _frequency * dt, 2 * PI);
        if (_phase < 0) _phase += 2 * PI;  // Negative frequency
        if (!isfinite(_phase)) _phase = 0;
    }
    _started = true;
    _lastMicros = now;

    switch (_type) {
        // Constant output which is equal to _amplitude
        case WaveType::CONSTANT: {
            _signal = _amplitude;
        } break;

            // Square wave that is "_amplitude" in the first half of the period and 0 in the second half
        case WaveType::SQUARE: {
            _signal = _phase < PI ? _amplitude : 0;
        } break;

        // Sawtooth wave (Generated using fourier series)
        case WaveType::SAWTOOTH: {
            double y = 0;
            int n = _fourierTermCount;
            for (int i = 1; i <= n; i++) {
                float b_n = i % 2 == 0 ? -2.0 / i : 2.0 / i;                         // Coefficient of each term,
                                                                                     // alternates between 2/i and -2/i
                y += b_n * sin(-i * _phase);                                         // Sin part of each term
            }
            _signal = constrain(y / PI * abs(_amplitude), -abs(_amplitude), abs(_amplitude));
            // _signal -= _amplitude;
        } break;

        // Triangular wave (Generated using fourier series)
        case WaveType::TRIANGULAR: {
            double y = 0;
            int n = _fourierTermCount;
            for (int i = 1; i <= n; i++)
                y += (1 / pow((2 * i - 1), 2) * cos((2 * i - 1) * _phase));
            y *= (8 * abs(_amplitude) / PI / PI);
            _signal = y;
        } break;

        // Sinusoidal wave
        case WaveType::SINUSOIDAL: {
            _signal = abs(_amplitude) * sin(_phase);
        } break;
        case WaveType::TRAPEZOIDAL: {
            double y = 0;
            int counter = 0;
            int n = _fourierTermCount;
            for (int i = 1; i < n * 2; i += 2) {
                if (counter <= 1)
                    y += sin(_phase * i) / (i * i);
                else
                    y -= sin(_phase * i) / (i * i);
                counter++;
                counter %= 4;
            }
            _signal = _amplitude * 8 * sqrt(2) / (PI * PI) * y;
        } break;
        default: {
            _signal = 0;
        } break;
    }
    return _signal + _offset;
}

// Set a new type for the waveform generator. Returns the previous type.
WaveformGenerator::WaveType WaveformGenerator::setType(WaveType type) {
    WaveType temp = _type;
    _type = type;
    _signal = 0;
    return temp;
}

// Set a new frequency for the waveform generator. Returns the previous
// frequency.
float WaveformGenerator::setFrequency(float frequency) {
    float temp = _frequency;
    _frequency = frequency;
    _period = 1.0 / frequency;
    return temp;
}

// Set a new perios for the waveform generator. Returns the previous frequency.
float WaveformGenerator::setPeriod(float period) {
    float temp = _period;
    _period = period;
    _frequency = 1.0 / _period;
    return temp;
}

// Set a new amplitude for the waveform generator. Returns the previous
// amplitude.
float WaveformGenerator::setAmplitude(float amplitude) {
    float temp = _amplitude;
    _amplitude = amplitude;
    return temp;
}

// Set an offset for the waveform. Returns the previous offset.
float WaveformGenerator::setOffset(float offset) {
    float temp = _offset;
    _offset = offset;
    return temp;
}

// Set the number of terms to be included in the fourier sum while calculating
// the sawtooth and triangular wave. It is set to 100 by default. Returns the
// previous number.
int WaveformGenerator::setFourierTerms(int fourierTermCount) {
    int temp = _fourierTermCount;
    _fourierTermCount = fourierTermCount;
    return temp;
}

float WaveformGenerator::getFrequency() { return _frequency; }

float WaveformGenerator::getPeriod() { return _period; }

float WaveformGenerator::getAmplitude() { return _amplitude; }

float WaveformGenerator::getOffset() { return _offset; }

int WaveformGenerator::getFourierTerms() { return _fourierTermCount; }