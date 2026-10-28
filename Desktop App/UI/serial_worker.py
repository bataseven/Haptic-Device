from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QObject
import sys
import time
from enum import Enum
from .settings import serialSettings as st

if sys.platform == "win32":
    import ctypes
    from ctypes import wintypes

    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _kernel32.CreateFileW.restype = wintypes.HANDLE
    _kernel32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
                                      wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE]
    _kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    _INVALID_HANDLE = wintypes.HANDLE(-1).value


def tryOpeningPort(port: str) -> str:
    """
    Opens and closes the port the way QSerialPort opens it on Windows. Returns "" if it opened, otherwise the error.

    QSerialPort.open() keeps Python's GIL while it waits, so a port that takes seconds to open, e.g. a Bluetooth port
    whose device is not reachable, freezes every thread of the app. ctypes releases the GIL during the call, so only
    the calling thread waits. On other systems this returns "" and QSerialPort reports the errors.
    """
    if sys.platform != "win32":
        return ""
    path = QSerialPortInfo(port).systemLocation() or "\\\\.\\" + port
    GENERIC_READ_WRITE, OPEN_EXISTING, FILE_FLAG_OVERLAPPED = 0xC0000000, 3, 0x40000000
    handle = _kernel32.CreateFileW(path, GENERIC_READ_WRITE, 0, None, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, None)
    if handle == _INVALID_HANDLE:
        return ctypes.FormatError(ctypes.get_last_error())
    _kernel32.CloseHandle(handle)
    return ""


class ConnectionStatus(Enum):
    Connected = 0
    Disconnected = 1
    DeviceNotFound = 2
    ConnectionLost = 3


class SerialWorker(QObject):
    """
    Opens, reads and writes the serial port. SerialHandler runs it in its own thread, because opening a port can block
    for seconds, e.g. a Bluetooth port whose device is not reachable. QSerialPort must only be used from one thread,
    so the other threads use it only through the slots of this class, by emitting signals connected to them.
    """
    connectionStatusChanged = pyqtSignal(object)
    receivedMessage = pyqtSignal(str)
    sentMessage = pyqtSignal(str)
    inRate = pyqtSignal(float)
    infoMessages = pyqtSignal(str)

    def __init__(self, parent: QObject = None):
        super().__init__(parent)
        # The port is a child of the worker, so it moves to the worker's thread with it
        self.serial: QSerialPort = QSerialPort(self)
        self.serialInRate = 0
        self.lastSerialInTime = time.time()

    @pyqtSlot(str, int)
    def connectSerial(self, port: str, baudrate: int) -> None:
        if self.serial.isOpen():
            self.disconnectSerial()
            return

        # Find out first whether the port opens, without blocking the other threads. See tryOpeningPort().
        error = tryOpeningPort(port)
        if error:
            self.infoMessages.emit(
                f'<span style=\"font-size:8pt; font-weight:400; color:red;\" >Serial Connection: {error}</span>')
            self.connectionStatusChanged.emit(ConnectionStatus.DeviceNotFound)
            return

        self.serial.deleteLater()
        self.serial = QSerialPort(port, self)
        self.serial.setBaudRate(baudrate)
        self.serial.setDataBits(QSerialPort.Data8)
        self.serial.setParity(QSerialPort.NoParity)
        self.serial.setStopBits(QSerialPort.OneStop)
        self.serial.setFlowControl(QSerialPort.NoFlowControl)

        self.serial.readyRead.connect(self.readSerial)
        self.serial.errorOccurred.connect(self.errorSerial)
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

    @pyqtSlot()
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
            # Corrupted bytes are replaced instead of raising an error. The message is then shown as invalid.
            msg = msgInBytes.data().decode(errors="replace")
            self.receivedMessage.emit(msg)

    @pyqtSlot(str, bool)
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
            self.serial.close()
            self.connectionStatusChanged.emit(ConnectionStatus.ConnectionLost)
            self.serialInRate = 0
