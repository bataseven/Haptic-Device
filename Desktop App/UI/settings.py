from math import pi
class SerialSettings:
    PORT = '-'
    BAUDRATE = '-'
    SERIAL_START_CH = '<'
    SERIAL_END_CH = '>'
    SERIAL_SEPERATOR_CH = '#'
    # Encoder count to mm factors of the finger and the palm. These are only used until the device sends its own values
    # ("U" message) when the app connects, so change them in the firmware (HapticDevice.h).
    COUNT_TO_MM = 29.5 / 99510.0  # 26.1 / 107493.0
    COUNT_TO_MM2 = (6.8 * pi) / (12 * 298)
    # The device stops the motors if it receives no message for HOST_TIMEOUT_MS, e.g. when this app crashes.
    HOST_TIMEOUT_MS = 1000
    HEARTBEAT_INTERVAL_MS = 250

serialSettings = SerialSettings()