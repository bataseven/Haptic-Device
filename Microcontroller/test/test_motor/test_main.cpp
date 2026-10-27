#include <unity.h>

#include "MotorWithEncoder.h"

const uint8_t PHASE_PIN = 8;
const uint8_t ENABLE_PIN = 9;

void setUp(void) {
    fake::setMicros(0);
    fake::analogOutputs().clear();
}
void tearDown(void) {}

static int phaseOutput() { return fake::analogOutputs()[PHASE_PIN]; }

// Sets a force controller with the given gains. The motor's dead band is 0 by default.
static void setForceGains(MotorWithEncoder &motor, double Kp, double Ki, double Kd) {
    motor.setControllerMode(MotorWithEncoder::FORCE_CONTROL);
    motor.setKp(Kp);
    motor.setKi(Ki);
    motor.setKd(Kd);
}

// Calls update() the given number of times, 1 ms apart, while the encoder moves by "step" counts each time.
static void updateEvery1ms(MotorWithEncoder &motor, Encoder *encoder, int times, int32_t step) {
    for (int i = 0; i < times; i++) {
        encoder->position += step;
        fake::advanceMicros(1000);
        motor.update();
    }
}

void test_derivative_is_zero_for_a_constant_error_below_1(void) {
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    setForceGains(motor, 0, 0, 100);
    motor.setDesiredForce(0.5);
    motor.setForce(0);

    motor.go();  // The error changes from 0 to 0.5, so the derivative term is 100 * 0.5
    TEST_ASSERT_EQUAL_INT(50, phaseOutput());
    motor.go();  // The error stays 0.5, so the derivative term is 0
    TEST_ASSERT_EQUAL_INT(0, phaseOutput());
}

void test_integral_term_keeps_fractions(void) {
    // Kp keeps the output above 5. Below that the motor is released, which also resets the integral term.
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    setForceGains(motor, 10, 0.25, 0);
    motor.setDesiredForce(1);
    motor.setForce(0);

    for (int i = 0; i < 24; i++) motor.go();  // 10 * 1 + 24 * 0.25 * 1
    TEST_ASSERT_EQUAL_INT(16, phaseOutput());
}

void test_force_kp_scale_does_not_change_the_gain(void) {
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    setForceGains(motor, 100, 0, 0);
    motor.setForceKpScale(0.5);
    motor.setDesiredForce(1);
    motor.setForce(0);

    motor.go();
    TEST_ASSERT_EQUAL_INT(50, phaseOutput());
    TEST_ASSERT_EQUAL_FLOAT(100, motor.getKp());
}

void test_reading_the_gains_of_a_mode_keeps_the_controller_mode(void) {
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    motor.setControllerMode(MotorWithEncoder::POSITION_CONTROL);
    motor.setKp(1);
    motor.setControllerMode(MotorWithEncoder::VELOCITY_CONTROL);
    motor.setKp(2);
    motor.setControllerMode(MotorWithEncoder::FORCE_CONTROL);
    motor.setKp(3);

    TEST_ASSERT_EQUAL_FLOAT(1, motor.getKp(MotorWithEncoder::POSITION_CONTROL));
    TEST_ASSERT_EQUAL_FLOAT(2, motor.getKp(MotorWithEncoder::VELOCITY_CONTROL));
    TEST_ASSERT_EQUAL_FLOAT(3, motor.getKp(MotorWithEncoder::FORCE_CONTROL));
    TEST_ASSERT_EQUAL_INT(MotorWithEncoder::FORCE_CONTROL, motor.getControllerMode());
}

void test_velocity_of_a_moving_motor(void) {
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *encoder = fake::encoders().back();
    updateEvery1ms(motor, encoder, 200, 1);  // More than the 150 values of the moving average
    TEST_ASSERT_FLOAT_WITHIN(1, 1000, motor.getVel());
}

void test_setting_the_encoder_position_does_not_change_the_velocity(void) {
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *encoder = fake::encoders().back();
    updateEvery1ms(motor, encoder, 10, 0);
    motor.setEncoderPos(1000);
    updateEvery1ms(motor, encoder, 1, 0);
    TEST_ASSERT_EQUAL_INT(1000, motor.getPos());
    TEST_ASSERT_EQUAL_FLOAT(0, motor.getVel());
}

void test_resetting_the_encoder_position_does_not_change_the_velocity(void) {
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    motor.setFDMElementCount(5);
    Encoder *encoder = fake::encoders().back();
    encoder->position = 1000;
    updateEvery1ms(motor, encoder, 200, 0);  // Standing still for longer than the moving average of the velocity
    motor.resetEncoderPos();
    updateEvery1ms(motor, encoder, 1, 0);
    TEST_ASSERT_EQUAL_INT(0, motor.getPos());
    TEST_ASSERT_EQUAL_FLOAT(0, motor.getVel());
}

void test_acceleration_needs_5_positions(void) {
    // With the default 2 positions the acceleration formula would read outside the position history
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *encoder = fake::encoders().back();
    updateEvery1ms(motor, encoder, 10, 5);
    TEST_ASSERT_EQUAL_INT(0, motor.getAcc());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_derivative_is_zero_for_a_constant_error_below_1);
    RUN_TEST(test_integral_term_keeps_fractions);
    RUN_TEST(test_force_kp_scale_does_not_change_the_gain);
    RUN_TEST(test_reading_the_gains_of_a_mode_keeps_the_controller_mode);
    RUN_TEST(test_velocity_of_a_moving_motor);
    RUN_TEST(test_setting_the_encoder_position_does_not_change_the_velocity);
    RUN_TEST(test_resetting_the_encoder_position_does_not_change_the_velocity);
    RUN_TEST(test_acceleration_needs_5_positions);
    return UNITY_END();
}
