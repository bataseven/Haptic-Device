from math import pi
class SerialSettings:
    PORT = '-'
    BAUDRATE = '-'
    SERIAL_START_CH = '<'
    SERIAL_END_CH = '>'
    SERIAL_SEPERATOR_CH = '#'
    COUNT_TO_MM = 29.5 / 99510.0  # 26.1 / 107493.0
    COUNT_TO_MM2 = (6.8 * pi) / (12 * 298)

serialSettings = SerialSettings()