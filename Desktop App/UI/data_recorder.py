
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
            # Include current date in the file name
            self.fileName = "\\HapticDeviceData_" + self.portName + "_" +\
                time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
            self.fullPath = self.savePathEdit.text() + self.fileName
            with open(self.fullPath, "w") as file:
                file.write(
                    "Data Point, \
Time, \
Finger Position, \
Finger Velocity, \
Finger Control Mode, \
Finger PID Position P, \
Finger PID Position I, \
Finger PID Position D, \
Finger PID Velocity P, \
Finger PID Velocity I, \
Finger PID Velocity D, \
Finger PID Force P, \
Finger PID Force I, \
Finger PID Force D, \
Finger Wave Type, \
Finger Wave Amplitude, \
Finger Wave Frequency, \
Finger Wave Offset, \
Finger Wave Value, \
Palm Position, \
Palm Velocity, \
Palm Control Mode, \
Palm PID Position P, \
Palm PID Position I, \
Palm PID Position D, \
Palm PID Velocity P, \
Palm PID Velocity I, \
Palm PID Velocity D, \
Palm PID Force P, \
Palm PID Force I, \
Palm PID Force D, \
Palm Wave Type, \
Palm Wave Amplitude, \
Palm Wave Frequency, \
Palm Wave Offset, \
Palm Wave Value, \
Thumb Outer Force, \
Thumb Inner Force, \
Index Inner Force, \
Index Outer Force\n")
        except Exception as e:
            print(e)
            return

        self.dataPoints = 0
        # Deactivate browse button
        self.browseButton.setEnabled(False)
        self.saveDurationEdit.setEnabled(False)
        self.savePathEdit.setEnabled(False)
        self.recordButton.setText("Stop")

        self.recordTimer.timeout.connect(self.stopRecording)
        self.recordTimer.start(int(float(self.saveDurationEdit.text()) * 1000))
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
            f"{self.dataPoints + 1} data point" + ("s" if self.dataPoints + 1 > 1 else "") + f" saved to {self.fullPath}")
        dialog.setWindowTitle("Recording done")
        dialog.setWindowIcon(self.windowIcon())
        dialog.setStandardButtons(QMessageBox.Ok)
        dialog.setIcon(QMessageBox.Information)
        dialog.exec_()

    def appendToFile(self):
        if not self.isRecording:
            return

        # Note that the current time may not accurately represent the time at which the data was received.
        # Because this function is called after couple of signals were emited from different QObjects.
        currentTime = time.time()
        fingerPos = self.hapticDevice.fingerPos
        fingerVel = self.hapticDevice.fingerVel
        fingerControlMode = self.hapticDevice.fingerControlMode
        fingerGains = self.hapticDevice.fingerGains
        fingerWaveParams = self.hapticDevice.waveformParams[0]

        palmPos = self.hapticDevice.palmPos
        palmVel = self.hapticDevice.palmVel
        palmControlMode = self.hapticDevice.palmControlMode
        palmGains = self.hapticDevice.palmGains
        palmWaveParams = self.hapticDevice.waveformParams[1]

        forceValues = self.hapticDevice.forceValues

        with open(self.fullPath, "a") as file:
            file.write(f"\
{self.dataPoints}, \
{currentTime:.3f}, \
{fingerPos}, \
{fingerVel}, \
{fingerControlMode}, \
{','.join(str(value) for value in fingerGains)}, \
{','.join(str(value) for value in [*fingerWaveParams.values()])}, \
{palmPos}, \
{palmVel}, \
{palmControlMode}, \
{','.join(str(value) for value in palmGains)}, \
{','.join(str(value) for value in [*palmWaveParams.values()])}, \
{','.join([str(force) for force in forceValues])}\n")

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
