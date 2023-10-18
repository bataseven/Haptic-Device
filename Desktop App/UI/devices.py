from PyQt5.QtCore import QObject

class HapticDevice(QObject):
    def __init__(self) -> None:
        super().__init__()
        
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

        self.forceValues = 4 * [0]
        self.netForce = 0
