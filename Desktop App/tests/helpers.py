"""Shared setup of the tests: a Qt application that needs no screen, the import paths and a fake serial port."""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)
sys.path.insert(0, os.path.join(APP_DIR, "tools"))

from PyQt5.QtCore import QByteArray  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

app = QApplication.instance() or QApplication([])

# Run the serial worker in the GUI thread, so the tests do not have to wait for it
from UI.serial_handler import SerialHandler  # noqa: E402
SerialHandler.useWorkerThread = False


class FakeSerialPort:
    """Stands in for QSerialPort: records what is written and returns the given lines when read."""

    def __init__(self, lines=None):
        self.written = []
        self.lines = list(lines or [])
        self.open = True

    def isOpen(self):
        return self.open

    def write(self, data):
        self.written.append(data.decode())

    def waitForBytesWritten(self, msecs):
        return True

    def close(self):
        self.open = False

    def canReadLine(self):
        return bool(self.lines)

    def readLine(self):
        return QByteArray(self.lines.pop(0))

    def portName(self):
        return "FAKE"

    def baudRate(self):
        return 115200
