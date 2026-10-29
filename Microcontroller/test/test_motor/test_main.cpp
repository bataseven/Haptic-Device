#include <unity.h>

#include "MotorWithEncoder.h"

const uint8_t PHASE_PIN = 8;
const uint8_t ENABLE_PIN = 9;

void setUp(void) {
    fake::setMicros(0);
    fake::analogOutputs().clear();
    fake::digitalInputs().clear();
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

    motor.go();  // There is no previous error after a reset, so there is no derivative term
    TEST_ASSERT_EQUAL_INT(0, phaseOutput());
    motor.setForce(-0.5);
    motor.go();  // The error changes from 0.5 to 1, so the derivative term is 100 * 0.5
    TEST_ASSERT_EQUAL_INT(50, phaseOutput());
    motor.go();  // The error stays 1, so the derivative term is 0
    TEST_ASSERT_EQUAL_INT(0, phaseOutput());
}

void test_steady_state_needs_a_full_error_window(void) {
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    setForceGains(motor, 0, 0, 0);
    motor.setDesiredForce(1);
    motor.setForce(0.9);  // 10 percent error

    // The 20 errors of the window are 0 after a reset. They must not make the 10 percent error look like 0.5 percent.
    for (int i = 0; i < 20; i++) TEST_ASSERT_FALSE(motor.go());

    motor.setForce(0.99);  // 1 percent error
    motor.resetController();
    for (int i = 0; i < 19; i++) TEST_ASSERT_FALSE(motor.go());
    TEST_ASSERT_TRUE(motor.go());
}

void test_holding_a_position_keeps_the_force_controller_state(void) {
    MotorWithEncoder motor(PHASE_PIN, ENABLE_PIN, 3, 4);
    setForceGains(motor, 0, 0, 100);
    motor.setHoldGains(2, 0, 0);
    motor.setDesiredForce(0.5);
    motor.setForce(0);
    motor.go();

    motor.resetHold();
    motor.holdPosition(100);  // The motor is at 0
    TEST_ASSERT_EQUAL_INT(200, phaseOutput());
    TEST_ASSERT_EQUAL_INT(MotorWithEncoder::FORCE_CONTROL, motor.getControllerMode());

    motor.go();  // The force error is still 0.5, so the derivative term stays 0
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

// Both driver inputs high is the brake of the PWM driver mode
static bool isBraked(uint8_t pin1, uint8_t pin2) {
    return fake::analogOutputs()[pin1] == MotorWithEncoder::maxPulse &&
           fake::analogOutputs()[pin2] == MotorWithEncoder::maxPulse;
}

static void updateBothEvery1ms(MotorWithEncoder &m1, MotorWithEncoder &m2, int times) {
    for (int i = 0; i < times; i++) {
        fake::advanceMicros(1000);
        m1.update();
        m2.update();
    }
}

void test_shared_limit_switch_homing_only_zeros_the_homing_motor(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);
    Encoder *palmEncoder = fake::encoders().back();

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);

    fingerEncoder->position = 5000;
    palmEncoder->position = 1000;
    updateBothEvery1ms(palm, finger, 5);

    palm.home();
    TEST_ASSERT_TRUE(palm.isHoming());
    TEST_ASSERT_FALSE(finger.isHoming());

    // Palm moves toward the switch, then presses the shared switch
    updateBothEvery1ms(palm, finger, 10);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);  // Longer than the 30 ms debounce

    TEST_ASSERT_TRUE(palm.isLimitSwitchPressed());
    TEST_ASSERT_FALSE(finger.isLimitSwitchPressed());
    TEST_ASSERT_EQUAL_INT(5000, finger.getPos());

    // Palm backs off until the shared switch is released
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);

    TEST_ASSERT_FALSE(palm.isHoming());
    TEST_ASSERT_EQUAL_INT(0, palm.getPos());
    TEST_ASSERT_EQUAL_INT(5000, finger.getPos());
}

void test_shared_limit_switch_serializes_simultaneous_homing(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);
    Encoder *palmEncoder = fake::encoders().back();

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);

    fingerEncoder->position = 4000;
    palmEncoder->position = 1500;
    updateBothEvery1ms(palm, finger, 5);

    // Request homing on both motors at the same time
    finger.home();
    palm.home();
    TEST_ASSERT_TRUE(finger.isHoming());
    TEST_ASSERT_TRUE(palm.isHoming());

    // Finger homes first while palm waits braked
    updateBothEvery1ms(palm, finger, 5);
    TEST_ASSERT_TRUE(fake::analogOutputs()[ENABLE_PIN] > 0);
    TEST_ASSERT_TRUE(isBraked(14, 15));

    // Finger hits the shared limit switch and backs off
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_EQUAL_INT(1500, palm.getPos());

    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_FALSE(finger.isHoming());
    TEST_ASSERT_EQUAL_INT(0, finger.getPos());

    // Palm now automatically starts moving toward the shared switch
    TEST_ASSERT_TRUE(palm.isHoming());
    TEST_ASSERT_TRUE(fake::analogOutputs()[15] > 0);

    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);

    TEST_ASSERT_FALSE(palm.isHoming());
    TEST_ASSERT_EQUAL_INT(0, palm.getPos());
    TEST_ASSERT_EQUAL_INT(0, finger.getPos());
}

void test_shared_limit_switch_infers_motor_from_negative_encoder_motion(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);
    Encoder *palmEncoder = fake::encoders().back();

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);

    fingerEncoder->position = 1000;
    palmEncoder->position = 1000;
    updateBothEvery1ms(palm, finger, 5);

    // Finger moves negatively into its limit switch, then stalls against the switch during the 30 ms debounce
    fingerEncoder->position = 900;
    updateBothEvery1ms(palm, finger, 1);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);

    TEST_ASSERT_TRUE(finger.isLimitSwitchPressed());
    TEST_ASSERT_FALSE(palm.isLimitSwitchPressed());
    TEST_ASSERT_EQUAL_INT(0, finger.getPos());
    TEST_ASSERT_EQUAL_INT(1000, palm.getPos());

    // Release the switch and move finger away
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);
    fingerEncoder->position = 2000;
    updateBothEvery1ms(palm, finger, 5);

    // Now palm moves negatively into its limit switch
    palmEncoder->position = 950;
    updateBothEvery1ms(palm, finger, 1);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);

    TEST_ASSERT_TRUE(palm.isLimitSwitchPressed());
    TEST_ASSERT_FALSE(finger.isLimitSwitchPressed());
    TEST_ASSERT_EQUAL_INT(0, palm.getPos());
    TEST_ASSERT_EQUAL_INT(2000, finger.getPos());
}

void test_shared_limit_switch_infers_motor_from_negative_drive(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);
    Encoder *palmEncoder = fake::encoders().back();

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);

    fingerEncoder->position = 3000;
    palmEncoder->position = 800;
    updateBothEvery1ms(palm, finger, 5);

    // Drive palm negatively toward its limit switch
    palm.rotate(-1, 0.5);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);

    TEST_ASSERT_TRUE(palm.isLimitSwitchPressed());
    TEST_ASSERT_FALSE(finger.isLimitSwitchPressed());
    TEST_ASSERT_EQUAL_INT(0, palm.getPos());
    TEST_ASSERT_EQUAL_INT(3000, finger.getPos());
}

void test_shared_limit_switch_infers_motor_closer_to_lower_limit_when_both_move_negatively(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);
    Encoder *palmEncoder = fake::encoders().back();

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);
    finger.setSoftPosLimit(0, 160000);
    palm.setSoftPosLimit(550, 7200);

    // Finger is in the middle of its travel (80000 / 160000 = 50%), palm is near 0 (100 / 6650 = 1.5%)
    fingerEncoder->position = 80000;
    palmEncoder->position = 100;
    updateBothEvery1ms(palm, finger, 5);

    // Both encoders move negatively at the same time when the switch triggers
    fingerEncoder->position = 79900;
    palmEncoder->position = 50;
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);

    TEST_ASSERT_TRUE(palm.isLimitSwitchPressed());
    TEST_ASSERT_FALSE(finger.isLimitSwitchPressed());
    TEST_ASSERT_EQUAL_INT(0, palm.getPos());
    TEST_ASSERT_EQUAL_INT(79900, finger.getPos());
}

void test_shared_limit_switch_clears_other_motor_when_stuck_high_before_homing(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);
    Encoder *palmEncoder = fake::encoders().back();

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);

    fingerEncoder->position = 4200;
    palmEncoder->position = 900;
    updateBothEvery1ms(palm, finger, 5);

    // Shared switch is already HIGH at boot because finger is resting on its switch
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    palm.home();

    // Debounce the initial HIGH reading, then palm tries backing off for > 500 ms
    updateBothEvery1ms(palm, finger, 40);
    updateBothEvery1ms(palm, finger, 510);

    // Since the switch stayed HIGH, finger is now backing off (+1) to clear the shared switch while palm waits
    TEST_ASSERT_TRUE(finger.isHoming());
    TEST_ASSERT_TRUE(palm.isHoming());
    TEST_ASSERT_TRUE(fake::analogOutputs()[PHASE_PIN] > 0);
    TEST_ASSERT_TRUE(isBraked(14, 15));

    // Finger clears its switch, so the shared pin goes LOW; finger's encoder is NOT blindly reset to 0
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_FALSE(finger.isHoming());
    TEST_ASSERT_EQUAL_INT(4200, finger.getPos());
    TEST_ASSERT_TRUE(palm.isHoming());

    // Palm now drives (-1) to its own switch, hits HIGH, backs off to LOW, and finishes homing
    TEST_ASSERT_TRUE(fake::analogOutputs()[15] > 0);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);

    TEST_ASSERT_FALSE(palm.isHoming());
    TEST_ASSERT_EQUAL_INT(0, palm.getPos());
}

void test_shared_limit_switch_aborts_both_motors_if_stuck_high_past_clearing_timeout(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    MotorWithEncoder palm(14, 15, 21, 20);

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);

    // Switch stays stuck HIGH permanently
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    palm.home();

    // Debounce (40 ms) + palm MOVING_TO_ZERO timeout (510 ms) + finger CLEARING_SHARED_SWITCH timeout (510 ms)
    updateBothEvery1ms(palm, finger, 40);
    updateBothEvery1ms(palm, finger, 510);
    TEST_ASSERT_TRUE(finger.isHoming());
    updateBothEvery1ms(palm, finger, 510);

    // Both motors must abort homing and release their outputs so neither runs away
    TEST_ASSERT_FALSE(finger.isHoming());
    TEST_ASSERT_FALSE(palm.isHoming());
    // Only the palm was being homed; the finger only tried to clear the switch for it
    TEST_ASSERT_TRUE(palm.homingFailed());
    TEST_ASSERT_FALSE(finger.homingFailed());
    TEST_ASSERT_EQUAL_INT(0, fake::analogOutputs()[PHASE_PIN]);
    TEST_ASSERT_EQUAL_INT(0, fake::analogOutputs()[ENABLE_PIN]);
    TEST_ASSERT_EQUAL_INT(0, fake::analogOutputs()[14]);
    TEST_ASSERT_EQUAL_INT(0, fake::analogOutputs()[15]);
}

void test_shared_limit_switch_aborts_if_opposite_limit_reached_while_backing_off(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);
    finger.setSoftPosLimit(0, 160000);

    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    palm.home();
    updateBothEvery1ms(palm, finger, 40);
    updateBothEvery1ms(palm, finger, 510);
    TEST_ASSERT_TRUE(finger.isHoming());

    // Finger reaches its upper soft limit while trying to clear the switch
    fingerEncoder->position = 160000;
    updateBothEvery1ms(palm, finger, 1);

    TEST_ASSERT_FALSE(finger.isHoming());
    TEST_ASSERT_FALSE(palm.isHoming());
    TEST_ASSERT_EQUAL_INT(0, fake::analogOutputs()[PHASE_PIN]);
    TEST_ASSERT_EQUAL_INT(0, fake::analogOutputs()[ENABLE_PIN]);
    TEST_ASSERT_TRUE(palm.homingFailed());
}

void test_limit_switch_in_positive_direction_sets_encoder_to_upper_soft_limit(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);
    Encoder *palmEncoder = fake::encoders().back();

    // Finger switch mounted at +1 (upper end, 160000); palm switch mounted at -1 (lower end, 0)
    finger.setSoftPosLimit(0, 160000);
    finger.setLimitSwitchPin(SHARED_SWITCH_PIN, 1);
    palm.setSoftPosLimit(550, 7200);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN, -1);

    fingerEncoder->position = 10000;
    palmEncoder->position = 2000;
    updateBothEvery1ms(palm, finger, 5);

    finger.home();
    updateBothEvery1ms(palm, finger, 5);
    // Finger drives in +1 direction (PHASE_PIN > 0) toward its upper switch
    TEST_ASSERT_TRUE(fake::analogOutputs()[PHASE_PIN] > 0);
    TEST_ASSERT_EQUAL_INT(0, fake::analogOutputs()[ENABLE_PIN]);

    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    // Finger backs off in -1 direction (ENABLE_PIN > 0)
    TEST_ASSERT_TRUE(fake::analogOutputs()[ENABLE_PIN] > 0);

    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);

    TEST_ASSERT_FALSE(finger.isHoming());
    TEST_ASSERT_EQUAL_INT(160000, finger.getPos());
    TEST_ASSERT_EQUAL_INT(2000, palm.getPos());
}

void test_other_motor_is_kept_still_while_a_motor_homes(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);
    Encoder *palmEncoder = fake::encoders().back();

    finger.setSoftPosLimit(0, 160000);
    finger.setLimitSwitchPin(SHARED_SWITCH_PIN, -1);
    palm.setSoftPosLimit(550, 7200);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN, -1);

    // Finger rests near its switch and is driven toward it by its controller
    fingerEncoder->position = 3000;
    palmEncoder->position = 3000;
    updateBothEvery1ms(palm, finger, 5);
    finger.setControllerMode(MotorWithEncoder::POSITION_CONTROL);
    finger.setKp(1);
    finger.setDesiredPosition(0);

    palm.home();
    TEST_ASSERT_TRUE(finger.isSharedLimitSwitchBusy());
    TEST_ASSERT_FALSE(palm.isSharedLimitSwitchBusy());

    // The finger is braked while the palm homes and its controller cannot drive it, even if the finger still drifts
    // toward its switch (e.g. pushed hard by the user's hand)
    for (int i = 0; i < 10; i++) {
        palmEncoder->position -= 50;
        fingerEncoder->position -= 2;
        updateBothEvery1ms(palm, finger, 1);
        finger.go();
    }
    TEST_ASSERT_TRUE(isBraked(PHASE_PIN, ENABLE_PIN));

    // The palm presses the shared switch: the press belongs to the palm, the finger's position is kept
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_TRUE(palm.isLimitSwitchPressed());
    TEST_ASSERT_FALSE(finger.isLimitSwitchPressed());
    TEST_ASSERT_FALSE(finger.isHoming());
    TEST_ASSERT_EQUAL_INT(2980, finger.getPos());

    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_FALSE(palm.isHoming());
    TEST_ASSERT_EQUAL_INT(0, palm.getPos());
    TEST_ASSERT_EQUAL_INT(2980, finger.getPos());

    // The finger can move again once the palm is homed
    TEST_ASSERT_FALSE(finger.isSharedLimitSwitchBusy());
    finger.go();
    TEST_ASSERT_TRUE(fake::analogOutputs()[ENABLE_PIN] > 0);
}

void test_finger_still_homes_after_clearing_the_switch_for_the_palm(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);
    fingerEncoder->position = 4200;

    // Power on with the finger resting on its switch, then home the palm and the finger like initialize() does
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    palm.home();
    finger.home();
    updateBothEvery1ms(palm, finger, 40);
    updateBothEvery1ms(palm, finger, 510);

    // The palm cannot release the switch, so the finger clears it
    TEST_ASSERT_TRUE(fake::analogOutputs()[PHASE_PIN] > 0);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);

    // The finger did not forget that it has to home: it waits braked while the palm homes
    TEST_ASSERT_TRUE(finger.isHoming());
    TEST_ASSERT_TRUE(isBraked(PHASE_PIN, ENABLE_PIN));
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_FALSE(palm.isHoming());
    TEST_ASSERT_EQUAL_INT(0, palm.getPos());

    // Then the finger homes
    TEST_ASSERT_TRUE(finger.isHoming());
    TEST_ASSERT_TRUE(fake::analogOutputs()[ENABLE_PIN] > 0);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_FALSE(finger.isHoming());
    TEST_ASSERT_EQUAL_INT(0, finger.getPos());
    TEST_ASSERT_FALSE(finger.homingFailed());
    TEST_ASSERT_FALSE(palm.homingFailed());
}

void test_homing_a_motor_while_it_clears_the_switch_homes_it_afterwards(void) {
    const uint8_t SHARED_SWITCH_PIN = 2;
    MotorWithEncoder finger(PHASE_PIN, ENABLE_PIN, 3, 4);
    Encoder *fingerEncoder = fake::encoders().back();
    MotorWithEncoder palm(14, 15, 21, 20);

    finger.setLimitSwitchPin(SHARED_SWITCH_PIN);
    palm.setLimitSwitchPin(SHARED_SWITCH_PIN);
    fingerEncoder->position = 1000;
    updateBothEvery1ms(palm, finger, 5);

    // The finger moves onto its switch and stays there
    fingerEncoder->position = 900;
    updateBothEvery1ms(palm, finger, 1);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_TRUE(finger.isLimitSwitchPressed());

    // Homing the palm makes the finger clear the switch first; the finger is asked to home while it clears
    palm.home();
    TEST_ASSERT_TRUE(finger.isHoming());
    finger.home();
    updateBothEvery1ms(palm, finger, 5);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);

    // The palm homes, then the finger
    TEST_ASSERT_TRUE(finger.isHoming());
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_FALSE(palm.isHoming());

    TEST_ASSERT_TRUE(finger.isHoming());
    fingerEncoder->position = 500;
    fake::digitalInputs()[SHARED_SWITCH_PIN] = HIGH;
    updateBothEvery1ms(palm, finger, 35);
    fake::digitalInputs()[SHARED_SWITCH_PIN] = LOW;
    updateBothEvery1ms(palm, finger, 35);
    TEST_ASSERT_FALSE(finger.isHoming());
    TEST_ASSERT_EQUAL_INT(0, finger.getPos());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_derivative_is_zero_for_a_constant_error_below_1);
    RUN_TEST(test_steady_state_needs_a_full_error_window);
    RUN_TEST(test_holding_a_position_keeps_the_force_controller_state);
    RUN_TEST(test_integral_term_keeps_fractions);
    RUN_TEST(test_force_kp_scale_does_not_change_the_gain);
    RUN_TEST(test_reading_the_gains_of_a_mode_keeps_the_controller_mode);
    RUN_TEST(test_velocity_of_a_moving_motor);
    RUN_TEST(test_setting_the_encoder_position_does_not_change_the_velocity);
    RUN_TEST(test_resetting_the_encoder_position_does_not_change_the_velocity);
    RUN_TEST(test_acceleration_needs_5_positions);
    RUN_TEST(test_shared_limit_switch_homing_only_zeros_the_homing_motor);
    RUN_TEST(test_shared_limit_switch_serializes_simultaneous_homing);
    RUN_TEST(test_shared_limit_switch_infers_motor_from_negative_encoder_motion);
    RUN_TEST(test_shared_limit_switch_infers_motor_from_negative_drive);
    RUN_TEST(test_shared_limit_switch_infers_motor_closer_to_lower_limit_when_both_move_negatively);
    RUN_TEST(test_shared_limit_switch_clears_other_motor_when_stuck_high_before_homing);
    RUN_TEST(test_shared_limit_switch_aborts_both_motors_if_stuck_high_past_clearing_timeout);
    RUN_TEST(test_shared_limit_switch_aborts_if_opposite_limit_reached_while_backing_off);
    RUN_TEST(test_limit_switch_in_positive_direction_sets_encoder_to_upper_soft_limit);
    RUN_TEST(test_other_motor_is_kept_still_while_a_motor_homes);
    RUN_TEST(test_finger_still_homes_after_clearing_the_switch_for_the_palm);
    RUN_TEST(test_homing_a_motor_while_it_clears_the_switch_homes_it_afterwards);
    return UNITY_END();
}
