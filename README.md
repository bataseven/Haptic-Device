# Haptic Device


During my master's degree, I worked on a developing a haptic device for a virtual reality application. The device was designed to simulate the stiffness and the inertia of virtual objects. 
<table>
  <tr>
    <td><img src="Images/Description.png" width="800"></td>
  </tr>
</table>

<!-- put Theses.pdf link -->
You can find the detailed information in my [thesis](Thesis.pdf).

Paper where we evaluated the stiffness rendering performance of the device through sets of psychophysical experiments is to be published soon.

## Repository layout

| Folder | Contents |
|---|---|
| [Microcontroller](Microcontroller) | Firmware of the Teensy 4.0 on the device (PlatformIO project) |
| [Desktop App](Desktop%20App) | PyQt5 app to control the device, plot its data and record it |
| [Images](Images) | Images used in the READMEs |

## Getting started

1. Build and upload the firmware: see [Microcontroller/README.md](Microcontroller/README.md).
2. Install and run the desktop app: see [Desktop App/README.md](Desktop%20App/README.md).
3. Connect the device over USB, select its port in the app and click **Connect**.

The messages between the device and the app are listed in the [serial protocol](Microcontroller/README.md#serial-protocol) section of the firmware README.
