
from ast import Expression
from os.path import dirname, abspath
from math import pi as PI
import time
import re
import os
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import QPushButton, QMessageBox, QLineEdit, QWidget, QFileDialog, QGroupBox, QProgressBar
from PyQt5.QtCore import pyqtSlot,  QTimer, QProcess, QEventLoop, Qt, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5.QtGui import QDoubleValidator, QIntValidator, QTextCursor
from PyQt5.uic import loadUi  # type: ignore

if __name__ == "__main__":
    from devices import HapticDevice
else:
    from .devices import HapticDevice

# The header and the rows are both built from these lists so that the column names always match the values.
GAIN_COLUMNS = ["PID Position P", "PID Position I", "PID Position D",
                "PID Velocity P", "PID Velocity I", "PID Velocity D",
                "PID Force P", "PID Force I", "PID Force D"]
# (Key in HapticDevice.waveformParams, column name)
WAVE_COLUMNS = [('T', "Wave Type"), ('F', "Wave Frequency"), ('A', "Wave Amplitude"),
                ('O', "Wave Offset"), ('V', "Wave Value")]
FORCE_COLUMNS = ["Thumb Outer Force", "Thumb Inner Force", "Index Inner Force", "Index Outer Force"]


def motorColumns(motor: str) -> list:
    return [f"{motor} Position", f"{motor} Velocity", f"{motor} Control Mode"] + \
        [f"{motor} {name}" for name in GAIN_COLUMNS] + \
        [f"{motor} {name}" for _, name in WAVE_COLUMNS]


# "Time" is when the app received the values, "Device Time" is when the device read them (seconds since it started).
# "Device Time" is the last column so that the other columns keep their positions from older recordings.
HEADER = ["Data Point", "Time"] + motorColumns("Finger") + motorColumns("Palm") + FORCE_COLUMNS + ["Device Time"]
SEPARATOR = ", "


class DataRecorder(QWidget):
    recordingInProgress = pyqtSignal(bool)

    def __init__(self, hapticDevice: HapticDevice, portName: str, parent=None) -> None:
        super().__init__(parent=parent)
        loadUi(dirname(abspath(__file__)) + "/data_recorder.ui", self)
        self.fingerGroupBox: QGroupBox
        self.palmGroupBox: QGroupBox
        self.virtualObjectGroupBox: QGroupBox
        self.savePathEdit: QLineEdit
        self.browseButton: QPushButton
        self.saveDurationEdit: QLineEdit
        self.progressBar: QProgressBar
        self.recordButton: QPushButton
        self.hapticDevice = hapticDevice
        self.portName: str = portName
        self.dataPoints = 0
        self.setupUI()

        self.recordTimer = QTimer()
        self.recordTimer.timeout.connect(self.stopRecording)
        self.progressBarTimer = QTimer()
        self.progressBarTimer.timeout.connect(self.updateProgressBar)
        self.isRecording = False

    def setupUI(self):
        iconPath = dirname(abspath(__file__)) + "/images/assembled.png"
        self.icon = QIcon(iconPath)
        self.setWindowIcon(self.icon)
        self.setWindowTitle("Data Recorder")
        self.browseButton.clicked.connect(self.browse)
        self.recordButton.clicked.connect(self.record)
        self.savePathEdit.setText(os.getcwd())
        validator = QDoubleValidator(
            self, bottom=0.0, top=100.0, notation=QDoubleValidator.StandardNotation)
        self.saveDurationEdit.setValidator(validator)
        # Make the widget window size unchangeable
        self.setFixedSize(self.size())

    def record(self):
        if not self.isRecording:
            self.startRecording()
        else:
            self.stopRecording()

    def startRecording(self):
        if self.isRecording:
            return
        try:
            duration = float(self.saveDurationEdit.text())
        except ValueError:
            duration = 0
        if duration <= 0:
            QMessageBox.warning(self, "Data Recorder", "Enter the recording duration in seconds.")
            return
        try:
            # Include current date in the file name
            self.fileName = "HapticDeviceData_" + self.portName + "_" +\
                time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
            self.fullPath = os.path.join(self.savePathEdit.text(), self.fileName)
            with open(self.fullPath, "w") as file:
                file.write(SEPARATOR.join(HEADER) + "\n")
        except OSError as e:
            QMessageBox.warning(self, "Data Recorder", f"Could not create the recording file:\n{e}")
            return

        self.dataPoints = 0
        # Deactivate browse button
        self.browseButton.setEnabled(False)
        self.saveDurationEdit.setEnabled(False)
        self.savePathEdit.setEnabled(False)
        self.recordButton.setText("Stop")

        self.recordTimer.start(int(duration * 1000))
        self.progressBarTimer.start(100)

        self.isRecording = True
        self.recordingInProgress.emit(self.isRecording)

    def stopRecording(self):
        if not self.isRecording:
            return

        self.browseButton.setEnabled(True)
        self.saveDurationEdit.setEnabled(True)
        self.savePathEdit.setEnabled(True)
        self.recordButton.setText("Record")

        # Stop the timer if it is running
        if self.recordTimer.isActive():
            self.recordTimer.stop()
        if self.progressBarTimer.isActive():
            self.progressBar.setValue(0)
            self.progressBarTimer.stop()

        self.isRecording = False
        self.recordingInProgress.emit(self.isRecording)

        # Pop a message box to inform the user that the recording is done
        dialog = QMessageBox(parent=None)
        dialog.setText(
            f"{self.dataPoints} data point" + ("" if self.dataPoints == 1 else "s") + f" saved to {self.fullPath}")
        dialog.setWindowTitle("Recording done")
        dialog.setWindowIcon(self.windowIcon())
        dialog.setStandardButtons(QMessageBox.Ok)
        dialog.setIcon(QMessageBox.Information)
        dialog.exec_()

    @pyqtSlot(bool)
    def onSerialConnected(self, connected: bool):
        if not connected:
            self.stopRecording()
        self.recordButton.setEnabled(connected)

    @pyqtSlot()
    def appendToFile(self):
        if not self.isRecording:
            return

        # The current time includes the transfer and processing delays. Use the "Device Time" column for exact intervals.
        currentTime = time.time()
        device = self.hapticDevice
        values = [self.dataPoints, f"{currentTime:.3f}"]
        values += [device.fingerPos, device.fingerVel, device.fingerControlMode, *device.fingerGains]
        values += [device.waveformParams[0][key] for key, _ in WAVE_COLUMNS]
        values += [device.palmPos, device.palmVel, device.palmControlMode, *device.palmGains]
        values += [device.waveformParams[1][key] for key, _ in WAVE_COLUMNS]
        values += device.forceValues
        values.append(f"{device.deviceTime:.6f}")

        try:
            with open(self.fullPath, "a") as file:
                file.write(SEPARATOR.join(str(value) for value in values) + "\n")
        except OSError as e:  # e.g. the drive was removed
            self.stopRecording()
            QMessageBox.warning(self, "Data Recorder", f"Recording stopped. Could not write to the file:\n{e}")
            return

        self.dataPoints += 1

    def updateProgressBar(self):
        progress = (self.recordTimer.interval(
        ) - self.recordTimer.remainingTime()) / self.recordTimer.interval() * 100
        self.progressBar.setValue(int(progress))
        self.progressBarTimer.start(100)

    def browse(self):
        path = QFileDialog.getExistingDirectory(self, "Select Directory")
        if path != "":
            self.savePathEdit.setText(path)

    def closeEvent(self, event):
        self.stopRecording()
        event.accept()


if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    window = DataRecorder(None,"COM3")
    window.show()
    sys.exit(app.exec_())
