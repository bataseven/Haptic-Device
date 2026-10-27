// #define RAINBOWS
#ifdef RAINBOWS
#include <rainbows.h>
#else
// #define ENABLE_ROS  // Initializes and declares the variables required for ROS serial connection if defined. Connects
// and communicates with the ROS network. (Has not been used in a while, will definetely not work properly.)
// #define USE_SERIAL1 // Sets the serial connection to use Serial1 (Connected to bluetooth module).
#define SERIAL_BAUD_RATE 115200

#include <Bounce.h>
#include <Encoder.h>
#include <HapticDevice.h>
#include <ROSHandler.h>
#include <SerialHandler.h>
#include <SoftwareSerial.h>
#include <math.h>

HapticDevice *device;
#ifdef ENABLE_ROS
ROSHandler *ROShandler;
#else
#ifdef USE_SERIAL1
#define SERIAL_PORT Serial1  // Sets the serial connection to use Serial1 aka bluetooth module.
#else
#define SERIAL_PORT Serial
#endif
SerialHandler SH = SerialHandler();
#endif

// unsigned long ControlLoopTimer;
unsigned long loopTimer;
const int maxLoopIndex = 100;
unsigned long loopTimes[maxLoopIndex];
int loopTimeIndex = 0;

IntervalTimer controlLoopTimer;
volatile uint32_t controlLoopCount = 0;  // Incremented by the control loop, so loop() can see that it is running.

void ControlLoopRoutine() {
    device->fingerControlLoop();
    device->palmControlLoop();
    controlLoopCount++;
}

// Starts the watchdog of the microcontroller (WDOG1). If it is not fed for 1 second, it resets the microcontroller.
// During the reset the motor driver inputs are not driven, so the motors are not powered.
void startWatchdog() {
    CCM_CCGR3 |= CCM_CCGR3_WDOG1(3);  // Enable the clock of the watchdog
    WDOG1_WMCR = 0;                   // Disable the power down counter, which would reset after 16 seconds
    // The timeout is (WT + 1) * 0.5 seconds. "|=" keeps the SRS and WDA bits set, writing 0 to them resets the board.
    WDOG1_WCR |= WDOG_WCR_WT(1) | WDOG_WCR_WDE;
}

void feedWatchdog() {
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
}

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

#ifdef ENABLE_ROS
    device = new HapticDevice();
    device->initialize();
    ROShandler = new ROSHandler(device);
    ROShandler->initialize();
    ROShandler->WSG50.cmd.speed = 350;         // Set the velocity for the gripper.
    ROShandler->WSG50.cmd.mode = "auto_update";  // Gripper working mode.
#else
    SERIAL_PORT.begin(SERIAL_BAUD_RATE);
    device = new HapticDevice();

    SH.setSerial(SERIAL_PORT);
    device->setSerial(SERIAL_PORT);

    // Blink the LED yellow until the serial connection is established.
    while (!SERIAL_PORT) {
        if (millis() / 1000 % 2 == 0) {
            digitalWrite(13, LOW);
            device->led.setPixelColor(0, YELLOW);
        } else
            device->led.setPixelColor(0, BLACK);

        device->led.show();
        delay(1);
    }

    device->initialize();
    SH.setHapticDevice(device);
    SH.printGains();
    SH.printLimits();
    SH.printModes();
    // SH.setDebug(true);
    SH.setPrintFrequency(40);  // 40 // 35 // 40
#endif

    device->finger->resetEncoderPos();
    // device->palm->resetEncoderPos();

    controlLoopTimer.begin(ControlLoopRoutine, 1000000.0 / CONTROL_LOOP_FREQUENCY);
    controlLoopTimer.priority(254);

    startWatchdog();
}

void loop() {
    device->update();  // Update the encoders, the force readings and the LED status.

#ifdef ENABLE_ROS
    ROShandler->update();
#else
    SH.update();
#endif

#ifdef ENABLE_ROS
    device->desiredFingerForce =
        -ROShandler->WSG50.force;  // Desired force is the reaction force reading from the gripper.
    ROShandler->WSG50.cmd.pos =
        constrain(map((float)device->finger->getPos(), device->finger->getUpperSoftPosLimit(), 0, 7, 110), 7,
                  110);  // Set the target position for the gripper.

    device->desiredFingerForce = 0;

    if (ROShandler->WSG50.isInContact) {
        digitalWrite(13, HIGH);
        device->desiredFingerForce = -log(ROShandler->WSG50.force);
    } else {
        digitalWrite(13, LOW);
        device->desiredFingerForce = 0;
    }
#endif
    // Make the device blink in a specific pattern to indicate the loop is running properly.
    analogWrite(LED_BUILTIN, (millis() / 100 % 25 == 0 || ((millis() + 200) / 100) % 25 == 0) ? 127 : 0);

    // Feed the watchdog only if the control loop has also run since the last time, so it resets the board if either
    // this loop or the control loop stops.
    static uint32_t lastControlLoopCount = 0;
    if (controlLoopCount != lastControlLoopCount) {
        lastControlLoopCount = controlLoopCount;
        feedWatchdog();
    }
}
#endif
