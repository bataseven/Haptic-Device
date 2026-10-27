# Desktop App for Controlling the Haptic Device
This is a desktop application for controlling the haptic device. It is written in Python and uses the [PyQt5](https://pypi.org/project/PyQt5/) library for the GUI.
<!-- SS Image -->
![Desktop App Screenshot](../Images/interface.jpg)

## Installing

The app is used on Windows with Python 3.11. In this folder:

```
pip install -r requirements.txt
```

## Running

```
python app.pyw
```

You can also double-click `app.pyw` to run it without a console window. Unexpected errors are then shown in a dialog and written to `stderr-app.pyw` in the temporary folder (`%TEMP%`).

1. Connect the device over USB and flash the firmware first if needed (see [the firmware README](../Microcontroller/README.md)).
2. Select the port of the device and click **Connect**. The device sends its gains, limits and controller modes, and the app shows them.
3. Choose the motor (finger or palm) and the controller mode, then send gains, a waveform or a virtual object.
4. Click **Disconnect** or close the app when you're done.

While it is connected, the app sends a heartbeat to the device every 250 ms. If the device doesn't receive anything from the app for 1 second, for example because the app crashed, it stops the motors (the LED turns purple) and the app shows a message.

## Features

- **Motor status:** position and velocity of each motor in encoder counts and mm. Clicking a motor's name homes it.
- **Controller:** controller mode and PID gains of each motor. **STOP FINGER** and **STOP PALM** brake a motor until you release it.
- **Waveform:** type, frequency, amplitude and offset of the reference signal of each motor. Tick **Units in mm** to enter the amplitude and offset in mm instead of encoder counts.
- **Virtual object:** width, stiffness and damping of the object the finger touches.
- **Animator:** live plot of the received values.
- **File > Record Data...:** records the received values to a CSV file for a given duration.
- **File > Settings...:** message characters, count to mm factors and position limits.
- **Serial messages:** the received (RX) and sent (TX) messages. **Help > Show Serial Commands...** lists the message types, and the [firmware README](../Microcontroller/README.md#serial-protocol) describes them in detail.

## Recorded data

Each row of a recording is one status update from the device (40 per second). The columns are the data point number, the time (seconds since 1970, when the app received the update), then for the finger and the palm: position and velocity (encoder counts), controller mode, the 9 PID gains, and the waveform type, frequency, amplitude, offset and current value. The last 4 columns are the forces of the sensors (N).

Recordings made before October 2026 have the "Wave Amplitude" and "Wave Frequency" labels swapped in the header, and some have other header problems. To fix the header of old recordings (a `.bak` copy of each changed file is kept):

```
python tools/fix_recording_headers.py <folder with the recordings>
```
