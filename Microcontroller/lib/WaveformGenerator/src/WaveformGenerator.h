#ifndef WaveformGenerator_H
#define WaveformGenerator_H
#include "Arduino.h"

class WaveformGenerator {
 public:
  enum WaveType { CONSTANT,       // Constant value
                  SQUARE,         // Square shaped wave that oscillates between 0 and +amplitude
                  SAWTOOTH,       // Sawtooth shaped wave that oscillates around 0, between amplitude and - amplitude
                  TRIANGULAR,     // Triangular shaped wave that oscillates around 0, between amplitude and -amplitude
                  SINUSOIDAL,     // Sinusoidal shaped wave that oscillates around 0, between amplitude and -amplitude
                  TRAPEZOIDAL };  // Trapezoidal signal that osciallates around 0, between amplitude and -amplitude

  WaveformGenerator();
  WaveformGenerator(WaveType, float, float);
  float generate();
  WaveType setType(WaveType);
  float setFrequency(float);
  float setPeriod(float);
  float setAmplitude(float);
  float setOffset(float);
  int setFourierTerms(int);

  float getFrequency();
  float getPeriod();
  float getAmplitude();
  float getOffset();
  int getFourierTerms();

 private:
  unsigned long _signalTimer;
  unsigned long _stepSignalTimer;
  bool _stepChanger = false;
  WaveType _type = WaveType::SQUARE;
  float _frequency = 1;
  float _period = 1;
  float _amplitude = 100000;
  float _offset = 0;
  float _signal = 0;
  int _fourierTermCount = 100;
};
#endif