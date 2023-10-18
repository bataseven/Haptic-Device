from os.path import dirname, abspath
import time
import re
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtWidgets import QPushButton, QLabel, QLineEdit, QComboBox, QRadioButton, QTextEdit, QCheckBox, QWidget, QMessageBox, QFrame, QGridLayout
from PyQt5.QtCore import pyqtSlot,  QTimer, QEventLoop, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QDoubleValidator, QIntValidator, QTextCursor, QPixmap
from PyQt5.uic import loadUi  # type: ignore

from .devices import HapticDevice
from .settings import serialSettings as st
from .serial_worker import SerialWorker, ConnectionStatus
from .animator import QAnimator
from .collapsible_box import QCollapsibleBox


class SerialHandler(QWidget):
    newData = pyqtSignal()
    newLimits = pyqtSignal()
    serialConnected = pyqtSignal(bool)
    messageToSend = pyqtSignal(str)
    dataToPlot = pyqtSignal(str, float)

    def __init__(self, hapticDevice: HapticDevice, parent=None) -> None:
        super().__init__(parent=parent)
        self.setWindowTitle("SerialHandler")
        loadUi(dirname(abspath(__file__)) + '/serial_handler.ui', self)
        self.portsCombo: QComboBox
        self.baudrateEdit: QLineEdit
        self.refreshPortsButton: QPushButton
        self.connectSerialButton: QPushButton
        self.animatorButton: QPushButton

        self.animatorButton.clicked.connect(self.openAnimator)
        self.fingerLabel: QLabel
        self.palmLabel: QLabel
        self.debugButton: QPushButton
        self.debug = False
        self.fingerPosLabel: QLabel
        self.fingerPosLabelMm: QLabel
        self.palmPosLabel: QLabel
        self.palmPosLabelMm: QLabel
        self.fingerVelLabel: QLabel
        self.fingerVelLabelMm: QLabel
        self.palmVelLabel: QLabel
        self.palmVelLabelMm: QLabel
        self.fingerModeLabel: QLabel
        self.palmModeLabel: QLabel
        self.fingerTypeLabel: QLabel
        self.palmTypeLabel: QLabel
        self.fingerFreqLabel: QLabel
        self.palmFreqLabel: QLabel
        self.fingerAmpLabel: QLabel
        self.palmAmpLabel: QLabel
        self.fingerOffsetLabel: QLabel
        self.palmOffsetLabel: QLabel
        self.fingerWaveValueLabel: QLabel
        self.palmWaveValueLabel: QLabel

        self.forceLabel1: QLabel
        self.forceLabel2: QLabel
        self.forceLabel3: QLabel
        self.forceLabel4: QLabel
        self.forceLabel5: QLabel

        self.stopFingerButton: QPushButton
        self.stopPalmButton: QPushButton

        self.positionControlButton: QPushButton
        self.velocityControlButton: QPushButton
        self.forceControlButton: QPushButton
        
        self.unitCheckBox: QCheckBox
        self.conversionLabel: QLabel
        self.waveRadio1: QRadioButton
        self.waveRadio2: QRadioButton
        self.waveCombo: QComboBox

        self.freqEdit: QLineEdit
        self.ampEdit: QLineEdit
        self.ampUnitLabel: QLabel
        self.amp2mm: QLabel
        self.offsetEdit: QLineEdit
        self.offsetUnitLabel: QLabel
        self.offset2mm: QLabel
        self.waveSendButton: QPushButton
        self.freqSendButton: QPushButton
        self.ampSendButton: QPushButton
        self.offsetSendButton: QPushButton
        self.saveEEPROMButton: QPushButton

        self.widthEdit: QLineEdit
        self.stiffnessEdit: QLineEdit
        self.dampingEdit: QLineEdit
        self.objectSendButton: QPushButton

        self.gainsRadio1: QRadioButton
        self.gainsRadio2: QRadioButton
        self.KpEdit: QLineEdit
        self.KiEdit: QLineEdit
        self.KdEdit: QLineEdit
        self.KpSendButton: QPushButton
        self.KiSendButton: QPushButton
        self.KdSendButton: QPushButton

        self.messagesLayout : QGridLayout
        self.serialMessagesLabel: QLabel
        self.rec1Label: QLabel
        self.rec2Label: QLabel
        self.messagesEdit: QTextEdit
        self.infoMessagesEdit: QTextEdit
        self.rxCheckBox: QCheckBox
        self.txCheckBox: QCheckBox
        self.invalidCheckBox: QCheckBox
        self.autoScrollCheckBox: QCheckBox

        self.isSerialConnected = False
        self.testVel = 0
        self.palmPosPrev = 0
        self.timePrev = time.time()

        self.serialThread: QThread = QThread()
        self.serialWorker: SerialWorker = SerialWorker()
        self.serialWorker.moveToThread(self.serialThread)

        self.serialWorker.connectionStatusChanged.connect(
            lambda status: self.handleConnectionChanges(status))

        self.serialWorker.receivedMessage.connect(
            lambda message: self.processReceivedMessage(message))

        self.serialWorker.sentMessage.connect(
            lambda message: self.processSentMessage(message))

        self.serialWorker.inRate.connect(lambda rate: self.serialMessagesLabel.setText(
            f"SERIAL MESSAGES (RX: {int(rate)} Hz)"))

        self.serialWorker.infoMessages.connect(
            lambda msg: self.infoMessagesEdit.append(msg))

        self.messageToSend.connect(
            lambda msg: self.serialWorker.writeSerial(msg))

        self.portName = ""

        self.recordingIconTimer: QTimer = QTimer()

        # Put all serial related buttons in a list to enable/disable them
        self.serialButtons = [self.refreshPortsButton, self.animatorButton, self.stopFingerButton, self.stopPalmButton, self.positionControlButton, self.velocityControlButton, self.debugButton,
                              self.forceControlButton, self.waveSendButton, self.freqSendButton, self.ampSendButton, self.offsetSendButton, self.objectSendButton, self.KpSendButton, self.KiSendButton, self.KdSendButton, self.saveEEPROMButton]

        self.hapticDevice = hapticDevice
        self.refreshPortsButton.clicked.connect(self.refreshSerialPorts)
        self.refreshPortsButton.animateClick()
        self.connectSerialButton.clicked.connect(lambda: self.serialWorker.connectSerial(
            port=self.portsCombo.currentText(), baudrate=int(self.baudrateEdit.text())
            if self.baudrateEdit.text().isdigit() else 115200))

        self.baudrateEdit.returnPressed.connect(
            self.connectSerialButton.animateClick)

        self.stopFingerButton.clicked.connect(self.stopFinger)
        self.stopPalmButton.clicked.connect(self.stopPalm)

        self.debugButton.clicked.connect(self.setDebug)

        self.positionControlButton.clicked.connect(self.sendControllerMode)
        self.velocityControlButton.clicked.connect(
            lambda: self.sendControllerMode(self.gainsRadio1.isChecked()))
        self.forceControlButton.clicked.connect(self.sendControllerMode)

        self.unitCheckBox.stateChanged.connect(lambda state: self.conversionLabel.setText(
            "Encoder Counts" if state == Qt.Checked else "Millimeters"))
        self.unitCheckBox.stateChanged.connect(lambda state: self.offsetUnitLabel.setText(
            "Millimeters" if state == Qt.Checked else "Counts"))
        self.unitCheckBox.stateChanged.connect(lambda state: self.ampUnitLabel.setText(
            "Millimeters" if state == Qt.Checked else "Counts"))
        self.waveSendButton.clicked.connect(self.sendWaveType)
        self.freqSendButton.clicked.connect(
            lambda: self.sendWaveParameter(self.freqEdit, 'F'))
        self.ampSendButton.clicked.connect(
            lambda: self.sendWaveParameter(self.ampEdit, 'A'))
        self.offsetSendButton.clicked.connect(
            lambda: self.sendWaveParameter(self.offsetEdit, 'O'))

        self.objectSendButton.clicked.connect(self.sendPsuedoObject)

        self.KpSendButton.clicked.connect(
            lambda: self.sendGain(self.KpEdit, "P"))
        self.KiSendButton.clicked.connect(
            lambda: self.sendGain(self.KiEdit, "I"))
        self.KdSendButton.clicked.connect(
            lambda: self.sendGain(self.KdEdit, "D"))

        self.saveEEPROMButton.clicked.connect(lambda: QMessageBox.warning(
            self, "Not Implemented", "This feature is not implemented yet!", QMessageBox.Ok))

        self.gainsRadio1.clicked.connect(self.setGainPlaceholders)
        self.gainsRadio2.clicked.connect(self.setGainPlaceholders)

        self.ampEdit.textChanged.connect(
            lambda text: self.setUnitsText(text, self.amp2mm))
        self.offsetEdit.textChanged.connect(
            lambda text: self.setUnitsText(text, self.offset2mm))

        self.waveRadio1.clicked.connect(
            lambda: self.ampEdit.textChanged.emit(self.ampEdit.text()))
        self.waveRadio1.clicked.connect(
            lambda: self.offsetEdit.textChanged.emit(self.offsetEdit.text()))
        self.waveRadio2.clicked.connect(
            lambda: self.ampEdit.textChanged.emit(self.ampEdit.text()))
        self.waveRadio2.clicked.connect(
            lambda: self.offsetEdit.textChanged.emit(self.offsetEdit.text()))
        self.unitCheckBox.stateChanged.connect(
            lambda: self.ampEdit.textChanged.emit(self.ampEdit.text()))
        self.unitCheckBox.stateChanged.connect(
            lambda: self.offsetEdit.textChanged.emit(self.offsetEdit.text()))

        def keyPressEventWrapper(self, func):
            def wrapper(*args, **kwargs):
                func(*args, **kwargs)
                if args[0].key() == Qt.Key_Return:
                    self.waveSendButton.animateClick()
            return wrapper
        self.waveCombo.keyPressEvent = keyPressEventWrapper(self,
                                                            self.waveCombo.keyPressEvent)

        # Create a timer to periodically update the serial ports when no port is connected.
        self.refreshPortsTimer = QTimer(self)
        self.refreshPortsTimer.timeout.connect(self.refreshSerialPorts)
        self.refreshPortsInterval = 100  # ms
        self.refreshPortsTimer.start(self.refreshPortsInterval)

        self.portsCombo.setEditable(True)
        self.portsCombo.lineEdit().returnPressed.connect(
            self.connectSerialButton.animateClick)
        self.portsCombo.setEditable(False)

        def keyPressEventWrapper(self, func):
            def wrapper(*args, **kwargs):
                func(*args, **kwargs)
                if args[0].key() == Qt.Key_Return:
                    self.connectSerialButton.animateClick()
            return wrapper

        self.portsCombo.keyPressEvent = keyPressEventWrapper(
            self, self.portsCombo.keyPressEvent)

        def mousePressEventWrapper(self, func, label, message=""):
            def wrapper(*args, **kwargs):
                func(*args, **kwargs)
                if not self.isSerialConnected:
                    return
                self.messageToSend.emit(message)
                label.setFrameShadow(QFrame.Sunken)
                QTimer.singleShot(
                    100, lambda: label.setFrameShadow(QFrame.Raised))
            return wrapper
        self.fingerLabel.mousePressEvent = mousePressEventWrapper(
            self, self.fingerLabel.mousePressEvent, self.fingerLabel, "H0")
        self.palmLabel.mousePressEvent = mousePressEventWrapper(
            self, self.palmLabel.mousePressEvent, self.palmLabel, "H1")

        # ! TODO: Not working properly.
        # Remove the messagesLayout from the main layout and add it to the collapsible box.
        # self.collapsibleMessageBox = QCollapsibleBox("Messages", self)
        # self.layout().removeItem(self.messagesLayout)
        # self.collapsibleMessageBox.setContentLayout(self.messagesLayout)
        # self.layout().addWidget(self.collapsibleMessageBox)
        
        self.setupUI()
        self.serialThread.start()
        self.serialThread.setPriority(QThread.TimeCriticalPriority)

    def setupUI(self) -> None:
        self.baudrateEdit.setValidator(
            QIntValidator(self, bottom=0, top=1000000000))

        self.freqEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))
        self.ampEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))
        self.offsetEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))
        self.widthEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))
        self.stiffnessEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))
        self.dampingEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))

        self.KpEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))
        self.KiEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))
        self.KdEdit.setValidator(QDoubleValidator(
            self, notation=QDoubleValidator.StandardNotation))
        self.setGainPlaceholders()

        imgPath = dirname(abspath(__file__)) + "/images/rec0.png"
        self.rec0 = QPixmap(imgPath)
        imgPath = dirname(abspath(__file__)) + "/images/rec1.png"
        self.rec1 = QPixmap(imgPath)
        imgPath = dirname(abspath(__file__)) + "/images/rec2.png"
        self.rec2 = QPixmap(imgPath)
        self.rec1Label.setPixmap(self.rec0)

        self.waveCombo.setItemData(
            0, "A constant amplitude value", Qt.ToolTipRole)
        self.waveCombo.setItemData(
            1, "Square shaped wave that oscillates between 0 and +amplitude", Qt.ToolTipRole)
        self.waveCombo.setItemData(
            2, "Sawtooth shaped wave that oscillates around 0, between +amplitude and - amplitude", Qt.ToolTipRole)
        self.waveCombo.setItemData(
            3, "Triangular shaped wave that oscillates around 0, between +amplitude and -amplitude", Qt.ToolTipRole)
        self.waveCombo.setItemData(
            4, "Sine shaped wave that oscillates around 0, between +amplitude and -amplitude", Qt.ToolTipRole)
        self.waveCombo.setItemData(
            5, "Trapezoidal signal that osciallates around 0, between +amplitude and -amplitude", Qt.ToolTipRole)

    def refreshSerialPorts(self) -> None:
        availablePorts = [port.portName()
                          for port in QSerialPortInfo.availablePorts()]
        for port in availablePorts:
            self.connectSerialButton.setEnabled(True)
            # Add the port to the combo box if it is not already there.
            if self.portsCombo.findText(port) == -1:
                self.portsCombo.addItem(port)
                self.portsCombo.setEnabled(True)

        for i in range(self.portsCombo.count()):
            # Remove the port from the combo box if it no longer exists.
            if self.portsCombo.itemText(i) not in availablePorts:
                self.portsCombo.removeItem(i)

        # If no ports are available, disable the connect button.
        if availablePorts == []:
            self.connectSerialButton.setEnabled(False)
            self.portsCombo.addItem("No ports detected")
            self.portsCombo.setEnabled(False)

    def handleConnectionChanges(self, status: ConnectionStatus) -> None:
        if status == ConnectionStatus.Disconnected:  # When the serial is closed
            self.connectSerialButton.setEnabled(False)
            self.serialMessagesLabel.setText("SERIAL MESSAGES")
            for button in self.serialButtons:
                button.setEnabled(False)
            self.connectSerialButton.setText("Disconnecting...")

            st.PORT = "-"
            st.BAUDRATE = "-"
            self.serialConnected.emit(False)
            self.isSerialConnected = False

            loop = QEventLoop()
            QTimer.singleShot(250, loop.quit)  # Wait 250ms to disconnect
            loop.exec_()

            self.connectSerialButton.setEnabled(True)
            self.connectSerialButton.setText("Connect")
            self.infoMessagesEdit.append("Serial Connection: Closed")

            self.stopFingerButton.setChecked(False)
            self.stopFingerButton.setText("STOP FINGER")

            self.stopPalmButton.setChecked(False)
            self.stopPalmButton.setText("STOP PALM")

            self.refreshPortsTimer.start(self.refreshPortsInterval)

        elif status == ConnectionStatus.Connected:  # When the serial is opened
            self.refreshPortsTimer.stop()
            self.connectSerialButton.setText("Disconnect")
            self.portName = self.serialWorker.serial.portName()
            self.baudrateEdit.setText(str(self.serialWorker.serial.baudRate()))
            st.PORT = self.portName
            st.BAUDRATE = str(self.serialWorker.serial.baudRate())
            self.serialConnected.emit(True)
            self.isSerialConnected = True

            for button in self.serialButtons:
                button.setEnabled(True)

        elif status == ConnectionStatus.ConnectionLost:
            self.connectSerialButton.setText("Connect")
            self.stopFingerButton.setChecked(False)
            self.stopFingerButton.setText("STOP FINGER")

            self.stopPalmButton.setChecked(False)
            self.stopPalmButton.setText("STOP PALM")

            for button in self.serialButtons:
                button.setEnabled(False)

            self.serialMessagesLabel.setText("SERIAL MESSAGES")

            st.PORT = "-"
            st.BAUDRATE = "-"
            self.serialConnected.emit(False)
            self.isSerialConnected = False

            self.refreshPortsTimer.start(self.refreshPortsInterval)

    @ pyqtSlot()
    def processReceivedMessage(self, msg: str) -> None:
        expression = re.compile(
            r'(' + st.SERIAL_START_CH + '(.*?)' + st.SERIAL_END_CH + ')')
        matches = expression.findall(msg)
        # TODO: Do not math the same message twice. (Here and in parse message)
        if self.rxCheckBox.isChecked() and len(matches) > 0:
            line = ""
            message = "<span style=\" font-size:8pt; font-weight:400; color:blue;\" >"
            for match in matches:
                line = (match[1] + "\t")
                message += "&lt;" + line + "&gt;"
            message += "</span>"

            topic = "<span style=\" font-size:8pt; font-weight:bold; color:blue;\" >"
            topic += "RX: "
            topic += "</span>"
            self.messagesEdit.append(topic + message)

            if (self.autoScrollCheckBox.isChecked()):
                self.messagesEdit.moveCursor(QTextCursor.End)

        if self.invalidCheckBox.isChecked() and len(matches) == 0:
            topic = "<span style=\" font-size:8pt; font-weight:bold; color:red;\" >"
            topic += "INV: "
            topic += "</span>"
            replaced_msg = msg.replace("<", "&lt;")
            replaced_msg = replaced_msg.replace(">", "&gt;")
            message = "<span style=\" font-size:8pt; font-weight:400; color:red;\" >"
            message += replaced_msg
            message += "</span>"
            self.messagesEdit.append(topic + message)

            if (self.autoScrollCheckBox.isChecked()):
                self.messagesEdit.moveCursor(QTextCursor.End)

        self.parseMessage(msg)

    def processSentMessage(self, text: str) -> None:
        """
        Write a string to the serial port. Start and End characters are added automatically.
        """
        if self.txCheckBox.isChecked():
            topic = "<span style=\" font-size:8pt; font-weight:bold; color:green;\" >"
            topic += "TX: "
            topic += "</span>"

            message = "<span style=\" font-size:8pt; font-weight:400; color:green;\" >"
            message += "&lt;" + text + "&gt;"
            message += "</span>"
            self.messagesEdit.append(topic + message)

            if (self.autoScrollCheckBox.isChecked()):
                self.messagesEdit.moveCursor(QTextCursor.End)

    def parseMessage(self, line: str) -> None:
        line = line.strip()

        # Find the matching regex groups for the expression: (@"<(.*?)>") # expression = re.compile(r'(<(.*?)>)')
        expression = re.compile(
            r'(' + st.SERIAL_START_CH + '(.*?)' + st.SERIAL_END_CH + ')')
        matches = expression.findall(line)

        # If there are no matches, return
        if len(matches) == 0:
            print(line)
            return

        self.newData.emit()
        for i, match in enumerate(matches):
            msg = match[0]
            if not (msg[0] == st.SERIAL_START_CH and msg[-1] == st.SERIAL_END_CH) and not self.debug:
                print("Invalid message: " + msg)
                return
            msg = msg[1:-1]
            msgType = msg[0]
            msg = msg[1:]
            if msgType == 'K':
                try:
                    motorIndex = int(msg[0])
                    msg = msg[1:]
                    if motorIndex == 0:
                        self.hapticDevice.fingerGains = [
                            float(k) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                    elif motorIndex == 1:
                        self.hapticDevice.palmGains = [
                            float(k) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                except Exception as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Gains, " + str(e))
                self.setGainPlaceholders()
            elif msgType == 'L':
                try:
                    limitType = msg[0]
                    msg = msg[1:]
                    motorIndex = int(msg[0])
                    msg = msg[1:]
                    if motorIndex == 0:
                        if limitType == 'S':
                            self.hapticDevice.fingerSoftLimits = [
                                float(k) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                        elif limitType == 'H':
                            self.hapticDevice.fingerHardLimits = [
                                float(k) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                    elif motorIndex == 1:
                        if limitType == 'S':
                            self.hapticDevice.palmSoftLimits = [
                                float(k) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                        elif limitType == 'H':
                            self.hapticDevice.palmHardLimits = [
                                float(k) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                    self.newLimits.emit()
                except Exception as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Limits, " + str(e))
            elif msgType == 'H':
                try:
                    motorIndex = int(msg[0])
                    msg = msg[1:]
                    if motorIndex == 0:
                        self.hapticDevice.fingerPos, self.hapticDevice.fingerVel = [
                            int(k, 16) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                    elif motorIndex == 1:
                        self.hapticDevice.palmPos, self.hapticDevice.palmVel = [
                            int(k, 16) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                except Exception as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Haptic, " + str(e))
                else:
                    if motorIndex == 0:
                        self.dataToPlot.emit(
                            "Finger Position", self.hapticDevice.fingerPos)
                        self.dataToPlot.emit(
                            "Finger Velocity", self.hapticDevice.fingerVel)
                    elif motorIndex == 1:
                        self.dataToPlot.emit(
                            "Palm Position", self.hapticDevice.palmPos)
                        self.dataToPlot.emit(
                            "Palm Velocity", self.hapticDevice.palmVel)

                self.setMotorStatusText()
            elif msgType == 'F':
                try:
                    self.hapticDevice.forceValues = [
                        float(k) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                except Exception as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Force, " + str(e))
                else:
                    self.dataToPlot.emit(
                        "Thumb Out", self.hapticDevice.forceValues[0])
                    self.dataToPlot.emit(
                        "Thumb In", self.hapticDevice.forceValues[1])
                    self.dataToPlot.emit(
                        "Index In", self.hapticDevice.forceValues[2])
                    self.dataToPlot.emit(
                        "Index Out", self.hapticDevice.forceValues[3])

            elif msgType == 'B':
                try:
                    self.debug = False if msg[0] == '0' else True
                except Exception as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Debug, " + str(e))
            elif msgType == 'N':
                try:
                    self.hapticDevice.netForce = float(msg)
                except ValueError as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Force, " + str(e))
                else:
                    self.dataToPlot.emit(
                        "Net Force", self.hapticDevice.netForce)
            elif msgType == 'Q':
                pass
            elif msgType == 'W':
                try:
                    generatorValue = int(msg[0])
                    msg = msg[1:]
                    self.hapticDevice.waveformParams[generatorValue]['V'] = float(
                        msg)
                except Exception as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Wave Value, " + str(e))
                else:
                    if generatorValue == 0:
                        self.dataToPlot.emit("Finger Wave Output",
                                             self.hapticDevice.waveformParams[0]['V'])
                    elif generatorValue == 1:
                        self.dataToPlot.emit("Palm Wave Output",
                                             self.hapticDevice.waveformParams[1]['V'])

                self.setWaveStatusText()
            if msgType == 'N' or msgType == 'F':
                self.setForceStatusText()

    def setGainPlaceholders(self) -> None:
        self.KpEdit.setText("")
        self.KiEdit.setText("")
        self.KdEdit.setText("")
        if self.gainsRadio1.isChecked():
            self.KpEdit.setPlaceholderText(
                f"{self.hapticDevice.fingerGains[self.hapticDevice.fingerControlMode * 3 + 0]:.3f}")
            self.KiEdit.setPlaceholderText(
                f"{self.hapticDevice.fingerGains[self.hapticDevice.fingerControlMode * 3 + 1]:.5f}")
            self.KdEdit.setPlaceholderText(
                f"{self.hapticDevice.fingerGains[self.hapticDevice.fingerControlMode * 3 + 2]:.3f}")
        elif self.gainsRadio2.isChecked():
            self.KpEdit.setPlaceholderText(
                f"{self.hapticDevice.palmGains[self.hapticDevice.palmControlMode * 3 + 0]:.3f}")
            self.KiEdit.setPlaceholderText(
                f"{self.hapticDevice.palmGains[self.hapticDevice.palmControlMode * 3 + 1]:.5f}")
            self.KdEdit.setPlaceholderText(
                f"{self.hapticDevice.palmGains[self.hapticDevice.palmControlMode * 3 + 2]:.3f}")

    def stopFinger(self) -> None:
        stop = "0"
        if not self.stopFingerButton.isChecked():
            self.stopFingerButton.setText("STOP FINGER")
            stop = "0"
        else:
            self.stopFingerButton.setText("RELEASE FINGER")
            stop = "1"

        msg = f"S0{stop}"
        # self.serialWorker.writeSerial(msg)
        self.messageToSend.emit(msg)

    def stopPalm(self) -> None:
        stop = "0"
        if not self.stopPalmButton.isChecked():
            self.stopPalmButton.setText("STOP PALM")
            stop = "0"
        else:
            self.stopPalmButton.setText("RELEASE PALM")
            stop = "1"

        msg = f"S1{stop}"
        # self.serialWorker.writeSerial(msg)
        self.messageToSend.emit(msg)

    @ pyqtSlot()
    def sendControllerMode(self, promptWarning=False) -> None:
        if promptWarning:
            reply = QMessageBox.question(self, 'Controller Mode',
                                         "Are you sure you want to change the controller mode? It may cause the motor to move unstably.",
                                         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.No:
                return
        self.KpEdit.setText("")
        self.KiEdit.setText("")
        self.KdEdit.setText("")
        motorIndex = "0"
        if self.gainsRadio1.isChecked():
            motorIndex = "0"
        elif self.gainsRadio2.isChecked():
            motorIndex = "1"

        controllerMode = "0"
        if(self.sender().objectName() == "positionControlButton"):
            controllerMode = "0"
            if motorIndex == "0":
                self.hapticDevice.fingerControlMode = 0
            elif motorIndex == "1":
                self.hapticDevice.palmControlMode = 0
        elif(self.sender().objectName() == "velocityControlButton"):
            controllerMode = "1"
            if motorIndex == "0":
                self.hapticDevice.fingerControlMode = 1
            elif motorIndex == "1":
                self.hapticDevice.palmControlMode = 1
        elif(self.sender().objectName() == "forceControlButton"):
            controllerMode = "2"
            if motorIndex == "0":
                self.hapticDevice.fingerControlMode = 2
            elif motorIndex == "1":
                self.hapticDevice.palmControlMode = 2

        self.setMotorStatusText()
        self.setGainPlaceholders()
        # self.serialWorker.writeSerial("O" + motorIndex + controllerMode)
        self.messageToSend.emit("O" + motorIndex + controllerMode)

    @ pyqtSlot()
    def sendWaveType(self) -> None:
        value = self.waveCombo.currentIndex()
        if self.waveRadio1.isChecked():
            generatorIndex = "0"
            self.hapticDevice.waveformParams[0]['T'] = self.waveCombo.currentText(
            )
        else:
            generatorIndex = "1"
            self.hapticDevice.waveformParams[1]['T'] = self.waveCombo.currentText(
            )
        self.setWaveStatusText()
        msg = f"WT{generatorIndex}{st.SERIAL_SEPERATOR_CH}{value}"
        # self.serialWorker.writeSerial(msg)
        self.messageToSend.emit(msg)

    @ pyqtSlot()
    def sendWaveParameter(self, textEdit: QTextEdit, paramType):
        value = textEdit.text()
        textEdit.setText("")
        # Convert from millimeters to count if the checkbox is checked and the parameter is amplitude or offset
        makeUnitConversion = self.unitCheckBox.isChecked() and (
            paramType == 'A' or paramType == 'O')
        if value == "":
            return
        if self.waveRadio1.isChecked():
            generatorIndex = "0"
            self.hapticDevice.waveformParams[0][paramType] = value if not makeUnitConversion else int(
                round(float(value) / st.COUNT_TO_MM))
        else:
            generatorIndex = "1"
            self.hapticDevice.waveformParams[1][paramType] = value if not makeUnitConversion else int(
                round(float(value) / st.COUNT_TO_MM2))
        self.setWaveStatusText()
        msg = f"W{paramType}{generatorIndex}{st.SERIAL_SEPERATOR_CH}{self.hapticDevice.waveformParams[int(generatorIndex)][paramType]}"
        # self.serialWorker.writeSerial(msg)
        self.messageToSend.emit(msg)

    @ pyqtSlot()
    def sendGain(self, textEdit: QTextEdit, gainType) -> None:
        value = textEdit.text()
        if value == "":
            return
        if self.gainsRadio1.isChecked():
            motorIndex = "0"
        else:
            motorIndex = "1"
        msg = f"K{gainType}{motorIndex}{st.SERIAL_SEPERATOR_CH}{value}"
        # self.serialWorker.writeSerial(msg)
        self.messageToSend.emit(msg)

    @ pyqtSlot()
    def sendPsuedoObject(self) -> None:
        width = self.widthEdit.text() if self.widthEdit.text() != "" else "0"
        stiffness = self.stiffnessEdit.text() if self.stiffnessEdit.text() != "" else "0"
        damping = self.dampingEdit.text() if self.dampingEdit.text() != "" else "0"

        msg = f"P{width}{st.SERIAL_SEPERATOR_CH}{stiffness}{st.SERIAL_SEPERATOR_CH}{damping}"
        # self.serialWorker.writeSerial(msg)
        self.messageToSend.emit(msg)

    def setMotorStatusText(self):
        self.fingerPosLabel.setText(str(int(self.hapticDevice.fingerPos)))
        self.fingerVelLabel.setText(str(int(self.hapticDevice.fingerVel)))

        self.fingerPosLabelMm.setText(
            f"{(self.hapticDevice.fingerPos * st.COUNT_TO_MM):.2f}")
        self.fingerVelLabelMm.setText(
            f"{(self.hapticDevice.fingerVel * st.COUNT_TO_MM):.2f}")

        self.palmPosLabel.setText(str(int(self.hapticDevice.palmPos)))
        self.palmVelLabel.setText(str(int(self.hapticDevice.palmVel)))

        self.palmPosLabelMm.setText(
            f"{(self.hapticDevice.palmPos * st.COUNT_TO_MM2):.2f}")
        self.palmVelLabelMm.setText(
            f"{(self.hapticDevice.palmVel * st.COUNT_TO_MM2):.2f}")

        if self.hapticDevice.fingerControlMode == 0:
            self.fingerModeLabel.setText("Position")
        elif self.hapticDevice.fingerControlMode == 1:
            self.fingerModeLabel.setText("Velocity")
        elif self.hapticDevice.fingerControlMode == 2:
            self.fingerModeLabel.setText("Force")

        if self.hapticDevice.palmControlMode == 0:
            self.palmModeLabel.setText("Position")
        elif self.hapticDevice.palmControlMode == 1:
            self.palmModeLabel.setText("Velocity")
        elif self.hapticDevice.palmControlMode == 2:
            self.palmModeLabel.setText("Force")

    def setWaveStatusText(self):
        self.fingerTypeLabel.setText(self.hapticDevice.waveformParams[0]['T'])
        self.fingerFreqLabel.setText(
            str(self.hapticDevice.waveformParams[0]['F']))
        self.fingerAmpLabel.setText(
            str(self.hapticDevice.waveformParams[0]['A']))
        self.fingerOffsetLabel.setText(
            str(self.hapticDevice.waveformParams[0]['O']))
        self.fingerWaveValueLabel.setText(
            str(self.hapticDevice.waveformParams[0]['V']))

        self.palmTypeLabel.setText(self.hapticDevice.waveformParams[1]['T'])
        self.palmFreqLabel.setText(
            str(self.hapticDevice.waveformParams[1]['F']))
        self.palmAmpLabel.setText(
            str(self.hapticDevice.waveformParams[1]['A']))
        self.palmOffsetLabel.setText(
            str(self.hapticDevice.waveformParams[1]['O']))
        self.palmWaveValueLabel.setText(
            str(self.hapticDevice.waveformParams[1]['V']))

    def setUnitsText(self, text: str, label: QLabel):
        txt = ""

        # This expression is used check the validity of the string as a number.
        expression = r'^[-+]?(?:\b[0-9]+(?:\.[0-9]*)?|\.[0-9]+\b)(?:[eE][-+]?[0-9]+\b)?$'

        # This lambda function returns None if the entered string does not correspond to a number.
        def regex(text): return re.match(expression, text)

        if regex(text) is not None:
            if self.waveRadio1.isChecked():
                if self.unitCheckBox.isChecked():
                    txt = f"{int(round(float(text) / st.COUNT_TO_MM))}" if st.COUNT_TO_MM != 0 else "Conversion factor is 0!"
                else:
                    txt = f"{float(text) * st.COUNT_TO_MM:.2f} mm"
            elif self.waveRadio2.isChecked():
                if self.unitCheckBox.isChecked():
                    txt = f"{int(round(float(text) / st.COUNT_TO_MM2))}" if st.COUNT_TO_MM2 != 0 else "Conversion factor is 0!"
                else:
                    txt = f"{float(text) * st.COUNT_TO_MM2:.2f} mm"

        label.setText(txt)

    def setForceStatusText(self):
        self.forceLabel1.setText(str(self.hapticDevice.forceValues[0]))
        self.forceLabel2.setText(str(self.hapticDevice.forceValues[1]))
        self.forceLabel3.setText(str(self.hapticDevice.forceValues[2]))
        self.forceLabel4.setText(str(self.hapticDevice.forceValues[3]))
        self.forceLabel5.setText(str(self.hapticDevice.netForce))

    def recording(self, bool):
        if bool:
            # Set rec1label ti rec1.png and rec2label to rec2.png
            self.rec1Label.setPixmap(self.rec1)
            self.rec2Label.setPixmap(self.rec2)
            self.showRec1 = False

            # Set the timer to timeout constantly
            self.recordingIconTimer = QTimer(self)
            self.recordingIconTimer.start(1000)
            self.recordingIconTimer.timeout.connect(self.blinkRecLabel)
        else:
            # Stop the timer
            self.showRec1 = False
            self.recordingIconTimer.stop()
            self.rec1Label.setPixmap(self.rec0)
            self.rec2Label.setPixmap(self.rec0)

    def blinkRecLabel(self):
        self.rec1Label.setPixmap(self.rec1 if self.showRec1 else self.rec0)
        self.showRec1 = not self.showRec1

    def openAnimator(self):
        self.animator = QAnimator()
        self.dataToPlot.connect(
            lambda label, data: self.animator.updateData(label, data))
            # self.serialWorker.connectionStatusChanged.connect(
        # lambda status: self.animator.close() if status != ConnectionStatus.Connected else None)
        self.newData.connect(self.animator.updateDataCount)
        self.serialWorker.inRate.connect(self.animator.setInRate)
        self.animator.destroyed.connect(self.dataToPlot.disconnect)
        # self.layout().addWidget(self.animator)

    def setDebug(self):
        if not self.debug:
            # self.serialWorker.writeSerial("B1")  # Set debug mode on
            self.messageToSend.emit("B1")  # Set debug mode on
        else:
            # self.serialWorker.writeSerial("B0")  # Set debug mode off
            self.messageToSend.emit("B0")  # Set debug mode off
