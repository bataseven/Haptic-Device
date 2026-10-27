from PyQt5.QtSerialPort import QSerialPort
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QObject
import time
from enum import Enum
from .settings import serialSettings as st


class ConnectionStatus(Enum):
    Connected = 0
    Disconnected = 1
    DeviceNotFound = 2
    ConnectionLost = 3


class SerialWorker(QObject):
    connectionStatusChanged = pyqtSignal(object)
    receivedMessage = pyqtSignal(str)
    sentMessage = pyqtSignal(str)
    inRate = pyqtSignal(float)
    infoMessages = pyqtSignal(str)

    def __init__(self, parent: QObject = None):
        super().__init__(parent)
        self.serial: QSerialPort = QSerialPort()
        self.serialInRate = 0
        self.lastSerialInTime = time.time()

    def connectSerial(self, port: str, baudrate: int) -> bool:
        if self.serial.isOpen():
            self.disconnectSerial()
            return

        try:
            self.serial = QSerialPort(port)
            self.serial.setBaudRate(baudrate)
        except Exception as e:
            self.infoMessages.emit("Error: " + str(e))
        self.serial.setDataBits(QSerialPort.Data8)
        self.serial.setParity(QSerialPort.NoParity)
        self.serial.setStopBits(QSerialPort.OneStop)
        self.serial.setFlowControl(QSerialPort.NoFlowControl)

        self.serial.readyRead.connect(self.readSerial)
        self.serial.errorOccurred.connect(lambda err: self.errorSerial(err))
        connected = self.serial.open(QSerialPort.ReadWrite)
        if not connected:
            self.connectionStatusChanged.emit(ConnectionStatus.DeviceNotFound)
            return
        self.connectionStatusChanged.emit(ConnectionStatus.Connected)
        self.infoMessages.emit(
            "<span style=\" font-size:8pt; font-weight:400; color:green;\" >Serial Connection: Opened (" + self.serial.portName() + ")</span>")
        self.serial.setDataTerminalReady(True)
        self.serial.flush()
        self.writeSerial("C1")
        # Ask the device to stop the motors if this app stops sending messages, e.g. when it crashes.
        self.writeSerial(f"T{st.HOST_TIMEOUT_MS}")

    def disconnectSerial(self) -> None:
        if not self.serial.isOpen():
            return
        self.serialInRate = 0
        self.writeSerial("T0")  # Disarm the timeout, so the device keeps working with the next program
        self.writeSerial("C0")
        self.serial.waitForBytesWritten(100)
        self.serial.close()
        self.connectionStatusChanged.emit(ConnectionStatus.Disconnected)

    @pyqtSlot()
    def readSerial(self):
        while self.serial.isOpen() and self.serial.canReadLine():
            msgInBytes = self.serial.readLine()
            # Calculate the time elapsed since the last message.

            instantaneousRate = 1 / \
                (time.time() - self.lastSerialInTime) if self.lastSerialInTime != time.time() else 0
            self.serialInRate = 0.9 * self.serialInRate + 0.1 * instantaneousRate
            self.inRate.emit(self.serialInRate)
            self.lastSerialInTime = time.time()
            msg = msgInBytes.data().decode()
            self.receivedMessage.emit(msg)

    def writeSerial(self, text: str, log: bool = True) -> None:
        if not self.serial.isOpen():
            self.infoMessages.emit("Error Sending: Not Connected")
            return
        self.serial.write(
            (st.SERIAL_START_CH + text + st.SERIAL_END_CH).encode())
        if log:
            self.sentMessage.emit(text)

    @pyqtSlot(QSerialPort.SerialPortError)
    def errorSerial(self, error: QSerialPort.SerialPortError):
        # Close serial port if QSerialPort.ResourceError
        if error != QSerialPort.NoError:
            self.infoMessages.emit(
                f'<span style=\"font-size:8pt; font-weight:400; color:red;\" >Serial Connection: {self.serial.errorString()}</span>')

        if error == QSerialPort.ResourceError:
            self.connectionStatusChanged.emit(ConnectionStatus.ConnectionLost)
            self.serial = QSerialPort()
            self.serialInRate = 0
