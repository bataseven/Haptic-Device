from PyQt5.QtCore import QObject

class HapticDevice(QObject):
    MICROS_OVERFLOW = 2 ** 32  # micros() of the device overflows every 71.6 minutes

    def __init__(self) -> None:
        super().__init__()

        # Device time of the latest status line in seconds since the device started. See setDeviceTime().
        self.deviceTime = 0.0
        self._lastDeviceMicros = None
        self._deviceMicrosOverflows = 0

        # 0: Position Control, 1: Velocity Control, 2: Force Control
        self.fingerControlMode = 2
        self.palmControlMode = 0  # 0: Position Control, 1: Velocity Control, 2: Force Control
        
        # Position P, Position I, Position D, Velocity P, Velocity I, Velocity D, Force P, Force I, Force D
        self.fingerGains = 9 * [0]
        # Position P, Position I, Position D, Velocity P, Velocity I, Velocity D, Force P, Force I, Force D
        self.palmGains = 9 * [0]

        # Motor related variables of the haptic device
        self.fingerPos = 0
        self.palmPos = 0
        self.fingerVel = 0
        self.palmVel = 0

        # Limits
        self.fingerSoftLimits = 2 * [0]
        self.fingerHardLimits = 2 * [0]
        self.palmSoftLimits = 2 * [0]
        self.palmHardLimits = 2 * [0]

        # Waveform related variables of the haptic device
        self.waveformParams = [{'T':'CONSTANT', 'F':0, 'A':0, 'O':0, 'V':0}, {'T':'CONSTANT', 'F':0, 'A':0, 'O':0, 'V':0}]

        # Virtual object parameters: width (counts), stiffness (N/mm), damping (N*s/mm)
        self.pseudoObject = {'width': 0, 'stiffness': 0, 'damping': 0}

        self.forceValues = 4 * [0]
        self.netForce = 0

    def setDeviceTime(self, micros: int) -> None:
        """
        Sets deviceTime from the micros() value sent by the device ("R" message).
        A value smaller than the previous one means micros() overflowed if the previous one was close to the overflow,
        otherwise the device was restarted.
        """
        if self._lastDeviceMicros is not None and micros < self._lastDeviceMicros:
            if self._lastDeviceMicros > self.MICROS_OVERFLOW - 60_000_000:
                self._deviceMicrosOverflows += 1
            else:
                self._deviceMicrosOverflows = 0
        self._lastDeviceMicros = micros
        self.deviceTime = (self._deviceMicrosOverflows * self.MICROS_OVERFLOW + micros) / 1e6
