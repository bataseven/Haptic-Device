#include <unity.h>

#include "WaveformGenerator.h"

void setUp(void) { fake::setMicros(0); }
void tearDown(void) {}

// Advances the time by the given microseconds, then generates the next value.
static float generateAfter(WaveformGenerator &generator, uint32_t microseconds) {
    fake::advanceMicros(microseconds);
    return generator.generate();
}

static void setUpSine(WaveformGenerator &generator, float frequency, float amplitude) {
    generator.setType(WaveformGenerator::SINUSOIDAL);
    generator.setFrequency(frequency);
    generator.setAmplitude(amplitude);
    generator.setOffset(0);
}

void test_constant_is_amplitude_plus_offset(void) {
    WaveformGenerator generator;
    generator.setType(WaveformGenerator::CONSTANT);
    generator.setAmplitude(3);
    generator.setOffset(2);
    TEST_ASSERT_EQUAL_FLOAT(5, generator.generate());
}

void test_sine_follows_time(void) {
    WaveformGenerator generator;
    setUpSine(generator, 1, 100);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, generator.generate());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 100, generateAfter(generator, 250000));
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, generateAfter(generator, 250000));
    TEST_ASSERT_FLOAT_WITHIN(0.01, -100, generateAfter(generator, 250000));
}

void test_square_is_amplitude_in_the_first_half_of_the_period(void) {
    WaveformGenerator generator;
    generator.setType(WaveformGenerator::SQUARE);
    generator.setFrequency(1);
    generator.setAmplitude(7);
    generator.setOffset(0);
    TEST_ASSERT_EQUAL_FLOAT(7, generator.generate());
    TEST_ASSERT_EQUAL_FLOAT(7, generateAfter(generator, 400000));  // 0.4 s
    TEST_ASSERT_EQUAL_FLOAT(0, generateAfter(generator, 200000));  // 0.6 s
    TEST_ASSERT_EQUAL_FLOAT(7, generateAfter(generator, 500000));  // 1.1 s
}

void test_changing_the_frequency_does_not_make_the_output_jump(void) {
    WaveformGenerator generator;
    setUpSine(generator, 1, 100);
    generator.generate();
    float before = generateAfter(generator, 100000);
    generator.setFrequency(5);
    float after = generateAfter(generator, 1000);  // 1 ms at 5 Hz moves the phase by only 0.03 rad
    TEST_ASSERT_FLOAT_WITHIN(5, before, after);
}

void test_zero_frequency_holds_the_output(void) {
    WaveformGenerator generator;
    setUpSine(generator, 1, 100);
    generator.generate();
    float held = generateAfter(generator, 250000);
    generator.setFrequency(0);
    TEST_ASSERT_FLOAT_WITHIN(0.01, held, generateAfter(generator, 1000000));
}

void test_micros_overflow_does_not_make_the_output_jump(void) {
    fake::setMicros(0xFFFFFFFF - 500);
    WaveformGenerator generator;
    setUpSine(generator, 1, 100);
    generator.generate();
    float before = generateAfter(generator, 100000);
    float after = generateAfter(generator, 1000);  // micros() overflows
    TEST_ASSERT_FLOAT_WITHIN(1, before, after);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_constant_is_amplitude_plus_offset);
    RUN_TEST(test_sine_follows_time);
    RUN_TEST(test_square_is_amplitude_in_the_first_half_of_the_period);
    RUN_TEST(test_changing_the_frequency_does_not_make_the_output_jump);
    RUN_TEST(test_zero_frequency_holds_the_output);
    RUN_TEST(test_micros_overflow_does_not_make_the_output_jump);
    return UNITY_END();
}
