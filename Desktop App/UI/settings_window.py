from os.path import dirname, abspath
from enum import Enum
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import QPushButton, QLabel, QLineEdit, QComboBox, QRadioButton, QTextEdit, QCheckBox, QWidget, QDialog, QMessageBox
from PyQt5.QtCore import pyqtSlot,  QTimer, QProcess, QEventLoop, Qt, pyqtSignal, QThread, QObject, QRegExp, QSettings
from PyQt5.QtGui import QDoubleValidator, QIntValidator, QTextCursor, QPixmap, QIcon, QRegExpValidator, QCloseEvent
from PyQt5.uic import loadUi  # type: ignore

from .settings import serialSettings as st
from .devices import HapticDevice


class SettingsWindow(QWidget):
    messageToSend = pyqtSignal(str)

    def __init__(self, hapticDevice: HapticDevice, parent=None) -> None:
        super().__init__(parent=parent)
        self.hapticDevice = hapticDevice
        self.setAttribute(Qt.WA_DeleteOnClose)
        loadUi(dirname(abspath(__file__)) + '\\settings_window.ui', self)
        self.portLabel: QLabel
        self.baudrateLabel: QLabel
        self.startChLabel: QLabel
        self.startChEdit: QLineEdit
        self.endChLabel: QLabel
        self.endChEdit: QLineEdit
        self.seperatorChLabel: QLabel
        self.seperatorChEdit: QLineEdit
        self.countToMmLabel: QLabel
        self.countToMmEdit: QLineEdit
        self.countToMmLabel2: QLabel
        self.countToMmEdit2: QLineEdit

        self.fingerSoftLowerEdit: QLineEdit
        self.fingerSoftUpperEdit: QLineEdit
        self.fingerSoftSendButton: QPushButton
        self.fingerHardLowerEdit: QLineEdit
        self.fingerHardUpperEdit: QLineEdit
        self.fingerHardSendButton: QPushButton
        self.palmSoftLowerEdit: QLineEdit
        self.palmSoftUpperEdit: QLineEdit
        self.palmSoftSendButton: QPushButton
        self.palmHardLowerEdit: QLineEdit
        self.palmHardUpperEdit: QLineEdit
        self.palmHardSendButton: QPushButton

        self.okButton: QPushButton
        self.cancelButton: QPushButton
        self.applyButton: QPushButton
        self.infoLabel: QLabel
        self.infoTimer: QTimer = QTimer()
        self.changeWasMade = False
        self.setupUI()
        

    def setupUI(self):
        self.setWindowTitle("Settings")
        self.icon = QIcon(dirname(abspath(__file__)) + "/images/settings.png")
        self.setWindowIcon(self.icon)

        # Set the text of the labels
        self.portLabel.setText(st.PORT)
        self.baudrateLabel.setText(str(st.BAUDRATE))
        self.startChEdit.setText(st.SERIAL_START_CH)
        self.endChEdit.setText(st.SERIAL_END_CH)
        self.seperatorChEdit.setText(st.SERIAL_SEPERATOR_CH)
        self.countToMmEdit.setText(f"{st.COUNT_TO_MM:.10f}")
        self.countToMmEdit2.setText(f"{st.COUNT_TO_MM2:.10f}")

        # Create the following regex expression: ^[ -~]{1}$ (This is a single character between space and ~)
        expression = r"^[ -~]{1}$"
        self.startChEdit.setValidator(QRegExpValidator(QRegExp(expression)))
        self.endChEdit.setValidator(QRegExpValidator(QRegExp(expression)))
        self.seperatorChEdit.setValidator(
            QRegExpValidator(QRegExp(expression)))

        self.countToMmEdit.setValidator(QDoubleValidator())
        self.countToMmEdit2.setValidator(QDoubleValidator())
        self.fingerSoftLowerEdit.setValidator(QIntValidator())
        self.fingerSoftUpperEdit.setValidator(QIntValidator())
        self.fingerHardLowerEdit.setValidator(QIntValidator())
        self.fingerHardUpperEdit.setValidator(QIntValidator())
        self.palmSoftLowerEdit.setValidator(QIntValidator())
        self.palmSoftUpperEdit.setValidator(QIntValidator())
        self.palmHardLowerEdit.setValidator(QIntValidator())
        self.palmHardUpperEdit.setValidator(QIntValidator())

        self.setLimitTexts()

        self.startChLabel.setToolTip(
            "This is the character that will be used to start a message.")
        self.startChEdit.setToolTip(
            "This is the character that will be used to start a message.")
        self.endChLabel.setToolTip(
            "This is the character that will be used to end a message.")
        self.endChEdit.setToolTip(
            "This is the character that will be used to end a message.")
        self.seperatorChLabel.setToolTip(
            "This is the character that will be used to seperate the data in a message.")
        self.seperatorChEdit.setToolTip(
            "This is the character that will be used to seperate the data in a message.")

        self.countToMmLabel.setToolTip(
            "This is the conversion factor from encoder count to mm for the finger motor.")
        self.countToMmEdit.setToolTip(
            "This is the conversion factor from encoder count to mm for the finger motor.")
        self.countToMmLabel2.setToolTip(
            "This is the conversion factor from encoder count to mm for the palm motor.")
        self.countToMmEdit2.setToolTip(
            "This is the conversion factor from encoder count to mm for the palm motor.")

        self.cancelButton.clicked.connect(self.close)
        self.okButton.clicked.connect(lambda: self.saveChanged(True))
        self.applyButton.clicked.connect(lambda: self.saveChanged(False))
        self.startChEdit.textChanged.connect(self.onTextChanged)
        self.endChEdit.textChanged.connect(self.onTextChanged)
        self.seperatorChEdit.textChanged.connect(self.onTextChanged)
        self.countToMmEdit.textChanged.connect(self.onTextChanged)
        self.countToMmEdit2.textChanged.connect(self.onTextChanged)
        self.fingerSoftSendButton.clicked.connect(lambda: self.sendLimit(
            'S', '0', self.fingerSoftLowerEdit.text(), self.fingerSoftUpperEdit.text()))
        self.fingerHardSendButton.clicked.connect(lambda: self.sendLimit(
            'H', '0', self.fingerHardLowerEdit.text(), self.fingerHardUpperEdit.text()))
        self.palmSoftSendButton.clicked.connect(lambda: self.sendLimit(
            'S', '1', self.palmSoftLowerEdit.text(), self.palmSoftUpperEdit.text()))
        self.palmHardSendButton.clicked.connect(lambda: self.sendLimit(
            'H', '1', self.palmHardLowerEdit.text(), self.palmHardUpperEdit.text()))

        self.fingerSoftLowerEdit.returnPressed.connect(self.fingerSoftSendButton.animateClick)
        self.fingerSoftUpperEdit.returnPressed.connect(self.fingerSoftSendButton.animateClick)
        self.fingerHardLowerEdit.returnPressed.connect(self.fingerHardSendButton.animateClick)
        self.fingerHardUpperEdit.returnPressed.connect(self.fingerHardSendButton.animateClick)
        self.palmSoftLowerEdit.returnPressed.connect(self.palmSoftSendButton.animateClick)
        self.palmSoftUpperEdit.returnPressed.connect(self.palmSoftSendButton.animateClick)
        self.palmHardLowerEdit.returnPressed.connect(self.palmHardSendButton.animateClick)
        self.palmHardUpperEdit.returnPressed.connect(self.palmHardSendButton.animateClick)

    def setLimitTexts(self):
        self.fingerSoftLowerEdit.setText(
            str(int(self.hapticDevice.fingerSoftLimits[0])))
        self.fingerSoftUpperEdit.setText(
            str(int(self.hapticDevice.fingerSoftLimits[1])))
        self.fingerHardLowerEdit.setText(
            str(int(self.hapticDevice.fingerHardLimits[0])))
        self.fingerHardUpperEdit.setText(
            str(int(self.hapticDevice.fingerHardLimits[1])))
        self.palmSoftLowerEdit.setText(
            str(int(self.hapticDevice.palmSoftLimits[0])))
        self.palmSoftUpperEdit.setText(
            str(int(self.hapticDevice.palmSoftLimits[1])))
        self.palmHardLowerEdit.setText(
            str(int(self.hapticDevice.palmHardLimits[0])))
        self.palmHardUpperEdit.setText(
            str(int(self.hapticDevice.palmHardLimits[1])))

    @pyqtSlot(bool)
    def saveChanged(self, closeWindow: bool):
        if self.changeWasMade:
            st.SERIAL_START_CH = self.startChEdit.text()
            st.SERIAL_END_CH = self.endChEdit.text()
            st.SERIAL_SEPERATOR_CH = self.seperatorChEdit.text()
            st.COUNT_TO_MM = float(self.countToMmEdit.text() or 0)
            st.COUNT_TO_MM2 = float(self.countToMmEdit2.text() or 0)
            self.applyButton.setEnabled(False)
        self.changeWasMade = False

        if closeWindow:
            self.close()
        else:
            self.infoLabel.setText("Settings applied.")
            self.infoTimer = QTimer(
                self, interval=2000, timeout=lambda: self.infoLabel.setText(""))
            self.infoTimer.setSingleShot(True)
            self.infoTimer.start()

    @pyqtSlot(str, str, str, str)
    def sendLimit(self, type: str, motorIndex: str, lower: str, upper: str):
        self.messageToSend.emit(
            f"L{type}{motorIndex}{int(lower) if lower != '' else '0'}{st.SERIAL_SEPERATOR_CH}{int(upper) if upper != '' else '0'}")

    def closeEvent(self, event: QCloseEvent):
        if self.changeWasMade:
            reply = QMessageBox.question(self, 'Unsaved Changes',
                                         "Do you want to save the unsaved changes?", QMessageBox.Yes |
                                         QMessageBox.No | QMessageBox.Cancel, QMessageBox.Cancel)
            if reply == QMessageBox.Yes:
                self.saveChanged(True)
            elif reply == QMessageBox.No:
                self.infoTimer.stop()
                self.changeWasMade = False
                event.accept()
            else:
                event.ignore()
        else:
            self.infoTimer.stop()
            self.changeWasMade = False
            event.accept()

    def onTextChanged(self):
        self.applyButton.setEnabled(True)
        self.changeWasMade = True
