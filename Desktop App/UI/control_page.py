from os.path import dirname, abspath
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QSpacerItem, QSizePolicy
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QFont, QIcon, QDoubleValidator, QIntValidator, QMouseEvent, QResizeEvent, QWheelEvent
from PyQt5.uic import loadUi  # type: ignore

from UI.serial_handler import SerialHandler
# from UI.tcp_handler import TCPHandler
from .devices import  HapticDevice

class ControlPage(QWidget):
    def __init__(self, hapticDevice:HapticDevice, parent=None) -> None:
        super().__init__(parent=parent)
        self.setLayout(QHBoxLayout())
        # They store the current state of the haptic device.
        self.hapticDevice = hapticDevice
        self.setupUI()
    
    def setupUI(self):
        self.serialHandler = SerialHandler(hapticDevice=self.hapticDevice, parent=self)
        self.layout().addWidget(self.serialHandler)




    