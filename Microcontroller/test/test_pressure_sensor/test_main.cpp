#include <unity.h>

#include "PressureSensor.h"

const uint8_t PIN = 1;

void setUp(void) { fake::analogInputs().clear(); }
void tearDown(void) {}

static void updateTimes(PressureSensor &sensor, int times) {
    for (int i = 0; i < times; i++) sensor.update();
}

void test_force_is_zero_until_calibrated(void) {
    fake::analogInputs()[PIN] = 500;
    PressureSensor sensor(PIN);
    sensor.update();
    TEST_ASSERT_FALSE(sensor.isCalibrated());
    TEST_ASSERT_EQUAL_FLOAT(0, sensor.getForce());
}

void test_calibrates_when_the_sensor_reads_zero(void) {
    // Without a load the sensor can read 0. The calibration must not wait until the sensor is first pressed,
    // otherwise that press becomes the zero.
    fake::analogInputs()[PIN] = 0;
    PressureSensor sensor(PIN);
    updateTimes(sensor, 300);
    TEST_ASSERT_TRUE(sensor.isCalibrated());
    TEST_ASSERT_EQUAL_FLOAT(0, sensor.getForce());

    fake::analogInputs()[PIN] = 200;
    updateTimes(sensor, 20);  // Fill the moving average window
    TEST_ASSERT_TRUE(sensor.getForce() > 0.1);
}

void test_force_at_startup_becomes_the_zero(void) {
    fake::analogInputs()[PIN] = 300;
    PressureSensor sensor(PIN);
    updateTimes(sensor, 300);
    TEST_ASSERT_TRUE(sensor.isCalibrated());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0, sensor.getForce());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_force_is_zero_until_calibrated);
    RUN_TEST(test_calibrates_when_the_sensor_reads_zero);
    RUN_TEST(test_force_at_startup_becomes_the_zero);
    return UNITY_END();
}
