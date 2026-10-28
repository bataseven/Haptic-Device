from os.path import dirname, abspath
import time
import re
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtWidgets import QApplication, QPushButton, QLabel, QLineEdit, QComboBox, QRadioButton, QTextEdit, QCheckBox, QWidget, QMessageBox, QFrame, QGridLayout, QShortcut
from PyQt5.QtCore import pyqtSlot, QTimer, Qt, pyqtSignal, QThread, QMetaObject, QSettings
from PyQt5.QtGui import QDoubleValidator, QIntValidator, QTextCursor, QPixmap, QKeySequence
from PyQt5.uic import loadUi  # type: ignore

from .devices import HapticDevice
from .settings import serialSettings as st
from .serial_worker import SerialWorker, ConnectionStatus
from .animator import QAnimator

TEENSY_VENDOR_ID = 0x16C0  # USB vendor id of the Teensy boards (PJRC)


def findMessages(text: str) -> list:
    """Returns (message with the start and end characters, message without them) for each message in the text."""
    expression = re.escape(st.SERIAL_START_CH) + '(.*?)' + re.escape(st.SERIAL_END_CH)
    return re.findall('(' + expression + ')', text)


class SerialHandler(QWidget):
    newData = pyqtSignal()
    newLimits = pyqtSignal()
    serialConnected = pyqtSignal(bool)
    messageToSend = pyqtSignal(str)
    dataToPlot = pyqtSignal(str, float)
    rxRate = pyqtSignal(float)
    # Requests to the serial worker, which runs in its own thread
    connectRequested = pyqtSignal(str, int)
    writeRequested = pyqtSignal(str, bool)
    disconnectRequested = pyqtSignal()

    # The tests set this to False, so the worker runs in the GUI thread and they do not have to wait for it
    useWorkerThread = True

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

        self.stopAllButton: QPushButton
        self.stopFingerButton: QPushButton
        self.homeFingerButton: QPushButton
        self.stopPalmButton: QPushButton
        self.homePalmButton: QPushButton

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
        self.waveSendAllButton: QPushButton

        self.widthLabel: QLabel
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
        self.gainsSendAllButton: QPushButton

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

        self.animator = None

        # The worker opens, reads and writes the port in its own thread, so a port that takes seconds to open does not
        # freeze the window. The signals between the two threads are connected to methods decorated with pyqtSlot,
        # so they run in the thread of their object.
        self.serialWorker = SerialWorker()
        self.workerThread = QThread(self)
        if self.useWorkerThread:
            self.serialWorker.moveToThread(self.workerThread)
            self.workerThread.start()
            QApplication.instance().aboutToQuit.connect(self.shutdown)

        self.serialWorker.connectionStatusChanged.connect(self.handleConnectionChanges)
        self.serialWorker.receivedMessage.connect(self.processReceivedMessage)
        self.serialWorker.sentMessage.connect(self.processSentMessage)
        self.serialWorker.inRate.connect(self.setInRate)
        self.serialWorker.infoMessages.connect(self.appendInfoMessage)

        self.connectRequested.connect(self.serialWorker.connectSerial)
        self.writeRequested.connect(self.serialWorker.writeSerial)
        self.disconnectRequested.connect(self.serialWorker.disconnectSerial)
        self.messageToSend.connect(self.sendMessage)

        self.portName = ""
        self.requestedBaudrate = 0
        self.isConnecting = False  # True while the worker opens a port
        self.refreshButtonWasEnabled = True

        self.recordingIconTimer = QTimer(self)
        self.recordingIconTimer.timeout.connect(self.blinkRecLabel)

        # Tells the device that this app is still running. The device stops the motors if the messages stop.
        self.heartbeatTimer = QTimer(self)
        self.heartbeatTimer.timeout.connect(lambda: self.writeRequested.emit("A", False))

        # Put all serial related buttons in a list to enable/disable them
        self.serialButtons = [self.refreshPortsButton, self.animatorButton, self.stopAllButton, self.stopFingerButton,
                              self.homeFingerButton, self.stopPalmButton, self.homePalmButton,
                              self.positionControlButton, self.velocityControlButton, self.debugButton,
                              self.forceControlButton, self.waveSendButton, self.freqSendButton, self.ampSendButton,
                              self.offsetSendButton, self.waveSendAllButton, self.objectSendButton, self.KpSendButton,
                              self.KiSendButton, self.KdSendButton, self.gainsSendAllButton]

        self.hapticDevice = hapticDevice
        self.refreshPortsButton.clicked.connect(self.refreshSerialPorts)
        self.refreshPortsButton.animateClick()
        self.connectSerialButton.clicked.connect(self.connectOrDisconnect)

        self.baudrateEdit.returnPressed.connect(
            self.connectSerialButton.animateClick)

        self.stopAllButton.clicked.connect(self.stopAll)
        self.stopAllShortcut = QShortcut(QKeySequence(Qt.Key_Escape), self)
        self.stopAllShortcut.activated.connect(self.onStopAllShortcut)
        self.stopFingerButton.clicked.connect(self.stopFinger)
        self.homeFingerButton.clicked.connect(self.homeFinger)
        self.stopPalmButton.clicked.connect(self.stopPalm)
        self.homePalmButton.clicked.connect(self.homePalm)

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
        self.unitCheckBox.stateChanged.connect(lambda state: self.widthLabel.setText(
            "Width [mm]: " if state == Qt.Checked else "Width [cnt]: "))
        self.unitCheckBox.stateChanged.connect(self.setObjectPlaceholders)
        self.waveSendButton.clicked.connect(self.sendWaveType)
        self.freqSendButton.clicked.connect(
            lambda: self.sendWaveParameter(self.freqEdit, 'F'))
        self.ampSendButton.clicked.connect(
            lambda: self.sendWaveParameter(self.ampEdit, 'A'))
        self.offsetSendButton.clicked.connect(
            lambda: self.sendWaveParameter(self.offsetEdit, 'O'))
        self.waveSendAllButton.clicked.connect(self.sendAllWaveParameters)

        self.objectSendButton.clicked.connect(self.sendPsuedoObject)

        self.KpSendButton.clicked.connect(
            lambda: self.sendGain(self.KpEdit, "P"))
        self.KiSendButton.clicked.connect(
            lambda: self.sendGain(self.KiEdit, "I"))
        self.KdSendButton.clicked.connect(
            lambda: self.sendGain(self.KdEdit, "D"))
        self.gainsSendAllButton.clicked.connect(self.sendAllGains)

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
        self.refreshPortsInterval = 1000  # ms
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
        self.setupUI()

    def setupUI(self) -> None:
        # Keep only the latest messages, so the logs don't grow without limit during long sessions
        self.messagesEdit.document().setMaximumBlockCount(1000)
        self.infoMessagesEdit.document().setMaximumBlockCount(1000)

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
        self.setObjectPlaceholders()

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

    @staticmethod
    def isTeensy(port: QSerialPortInfo) -> bool:
        return port.hasVendorIdentifier() and port.vendorIdentifier() == TEENSY_VENDOR_ID

    def refreshSerialPorts(self) -> None:
        # The combo box shows the description of each port, and keeps the port name as the item data.
        ports = {port.portName(): port for port in QSerialPortInfo.availablePorts()}

        # Remove the ports that no longer exist. Going backwards, so removing an item doesn't skip the next one.
        for i in reversed(range(self.portsCombo.count())):
            if self.portsCombo.itemData(i) not in ports:
                self.portsCombo.removeItem(i)

        # Add the ports that are not in the combo box yet.
        for name, port in ports.items():
            if self.portsCombo.findData(name) == -1:
                description = "Teensy" if self.isTeensy(port) else port.description()
                self.portsCombo.addItem(f"{name} ({description})" if description else name, name)
                # Select the device as soon as it is plugged in, unless another Teensy is already selected
                selected = ports.get(self.portsCombo.currentData())
                if self.isTeensy(port) and not (selected and self.isTeensy(selected)):
                    self.portsCombo.setCurrentIndex(self.portsCombo.count() - 1)

        # If no ports are available, disable the connect button.
        hasPorts = len(ports) > 0
        if not hasPorts:
            self.portsCombo.addItem("No ports detected")
        # While a port is being opened, they stay disabled until the worker reports the result
        if not self.isConnecting:
            self.portsCombo.setEnabled(hasPorts)
            self.connectSerialButton.setEnabled(hasPorts)

    @pyqtSlot(str)
    def sendMessage(self, text: str) -> None:
        self.writeRequested.emit(text, True)

    @pyqtSlot(float)
    def setInRate(self, rate: float) -> None:
        self.serialMessagesLabel.setText(f"SERIAL MESSAGES (RX: {int(rate)} Hz)")
        self.rxRate.emit(rate)

    @pyqtSlot(str)
    def appendInfoMessage(self, message: str) -> None:
        self.infoMessagesEdit.append(message)

    @pyqtSlot()
    def shutdown(self) -> None:
        """Closes the port and stops the worker thread. Called when the app closes."""
        if not self.workerThread.isRunning():
            self.serialWorker.disconnectSerial()
            return
        # Wait until the worker has sent the last messages and closed the port
        QMetaObject.invokeMethod(self.serialWorker, "disconnectSerial", Qt.BlockingQueuedConnection)
        self.workerThread.quit()
        self.workerThread.wait()

    def connectOrDisconnect(self) -> None:
        if self.isSerialConnected:
            self.disconnectRequested.emit()
            return
        port = self.portsCombo.currentData()
        if not port:
            return
        baudrate = int(self.baudrateEdit.text()) if self.baudrateEdit.text().isdigit() else 115200

        # The worker opens the port in its thread and reports the result with connectionStatusChanged. Opening can
        # take seconds, e.g. a Bluetooth port whose device is not reachable.
        self.isConnecting = True
        self.refreshPortsTimer.stop()
        self.connectSerialButton.setText("Connecting...")
        self.connectSerialButton.setEnabled(False)
        self.portsCombo.setEnabled(False)
        self.refreshButtonWasEnabled = self.refreshPortsButton.isEnabled()
        self.refreshPortsButton.setEnabled(False)
        self.portName = port
        self.requestedBaudrate = baudrate
        self.connectRequested.emit(port, baudrate)

    @pyqtSlot(object)
    def handleConnectionChanges(self, status: ConnectionStatus) -> None:
        if status == ConnectionStatus.Disconnected:  # When the serial is closed
            self.heartbeatTimer.stop()
            self.connectSerialButton.setEnabled(False)
            self.serialMessagesLabel.setText("SERIAL MESSAGES")
            for button in self.serialButtons:
                button.setEnabled(False)
            self.connectSerialButton.setText("Disconnecting...")

            st.PORT = "-"
            st.BAUDRATE = "-"
            self.serialConnected.emit(False)
            self.isSerialConnected = False

            # The worker has already sent the last messages and closed the port
            self.connectSerialButton.setEnabled(True)
            self.connectSerialButton.setText("Connect")
            self.infoMessagesEdit.append("Serial Connection: Closed")

            self.stopFingerButton.setChecked(False)
            self.stopFingerButton.setText("STOP FINGER")

            self.stopPalmButton.setChecked(False)
            self.stopPalmButton.setText("STOP PALM")

            self.refreshPortsTimer.start(self.refreshPortsInterval)

        elif status == ConnectionStatus.Connected:  # When the serial is opened
            self.isConnecting = False
            self.refreshPortsTimer.stop()
            self.connectSerialButton.setText("Disconnect")
            self.connectSerialButton.setEnabled(True)
            self.portsCombo.setEnabled(True)
            self.baudrateEdit.setText(str(self.requestedBaudrate))
            st.PORT = self.portName
            st.BAUDRATE = str(self.requestedBaudrate)
            self.serialConnected.emit(True)
            self.isSerialConnected = True
            self.heartbeatTimer.start(st.HEARTBEAT_INTERVAL_MS)

            for button in self.serialButtons:
                button.setEnabled(True)
            self.updateControllerModeButtons()

        elif status == ConnectionStatus.DeviceNotFound:  # When the port could not be opened
            self.isConnecting = False
            self.connectSerialButton.setText("Connect")
            self.refreshPortsButton.setEnabled(self.refreshButtonWasEnabled)
            self.refreshSerialPorts()  # Enables the button and the port list again
            self.refreshPortsTimer.start(self.refreshPortsInterval)
            self.infoMessagesEdit.append(
                f"<span style=\" font-size:8pt; font-weight:400; color:red;\" >Could not open "
                f"{self.portName}. Select the port of the haptic device: it is listed as Teensy "
                f"when it is connected over USB.</span>")

        elif status == ConnectionStatus.ConnectionLost:
            self.heartbeatTimer.stop()
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

    @pyqtSlot(str)
    def processReceivedMessage(self, msg: str) -> None:
        matches = findMessages(msg)
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

        self.parseMessage(msg, matches)

    @pyqtSlot(str)
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

    def parseMessage(self, line: str, matches: list = None) -> None:
        """Parses the messages in the line. Pass the result of findMessages(line) if it was already found."""
        line = line.strip()
        if matches is None:
            matches = findMessages(line)

        # If there are no matches, return
        if len(matches) == 0:
            print(line)
            return

        hasStatus = False
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
                    hasStatus = True
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
            elif msgType == 'M':
                # The controller mode of a motor. The device sends it when the app connects and when the mode changes.
                try:
                    motorIndex = int(msg[0])
                    mode = int(msg[1:])
                    if mode not in (0, 1, 2):
                        raise ValueError(f"unknown mode {mode}")
                    if motorIndex == 0:
                        self.hapticDevice.fingerControlMode = mode
                    elif motorIndex == 1:
                        self.hapticDevice.palmControlMode = mode
                except Exception as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Controller Mode, " + str(e))
                else:
                    self.setMotorStatusText()
                    self.setGainPlaceholders()
            elif msgType == 'R':
                try:
                    self.hapticDevice.setDeviceTime(int(msg))
                except ValueError as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Device Time, " + str(e))
            elif msgType == 'U':
                # Encoder count to mm factors of the finger and the palm. The firmware is the source of these values.
                try:
                    st.COUNT_TO_MM, st.COUNT_TO_MM2 = [
                        float(k) for k in msg.split(st.SERIAL_SEPERATOR_CH)]
                except Exception as e:
                    self.infoMessagesEdit.append(
                        "Error Parsing: Units, " + str(e))
                else:
                    self.setMotorStatusText()
                    self.setObjectPlaceholders()
            elif msgType == 'X':
                self.infoMessagesEdit.append(
                    f"<span style=\" font-size:8pt; font-weight:400; color:red;\" >The device stopped the motors because it "
                    f"received no message from this app for {msg} ms. Its waveforms and virtual object were reset.</span>")
                for params in self.hapticDevice.waveformParams:
                    params.update({'T': 'CONSTANT', 'F': 0, 'A': 0, 'O': 0})
                self.hapticDevice.pseudoObject.update({'width': 0, 'stiffness': 0, 'damping': 0})
                self.setWaveStatusText()
                self.setObjectPlaceholders()
                # The device disarms the timeout when it fires, so arm it again
                self.messageToSend.emit(f"T{st.HOST_TIMEOUT_MS}")
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

        # Emitted after the whole line is parsed so that listeners (e.g. the data recorder) get this line's values.
        # Lines without a status message (e.g. gains or limits) are not new data points.
        if hasStatus:
            self.newData.emit()

    def updateControllerModeButtons(self) -> None:
        isFinger = self.gainsRadio1.isChecked()
        mode = self.hapticDevice.fingerControlMode if isFinger else self.hapticDevice.palmControlMode
        self.positionControlButton.setChecked(mode == 0)
        self.velocityControlButton.setChecked(mode == 1)
        self.forceControlButton.setChecked(mode == 2 and isFinger)
        self.forceControlButton.setEnabled(self.isSerialConnected and isFinger)

    def setGainPlaceholders(self) -> None:
        self.KpEdit.setText("")
        self.KiEdit.setText("")
        self.KdEdit.setText("")
        self.updateControllerModeButtons()
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

    def setObjectPlaceholders(self) -> None:
        widthCounts = self.hapticDevice.pseudoObject['width']
        if self.unitCheckBox.isChecked():
            widthText = f"{widthCounts * st.COUNT_TO_MM:.2f}"
        else:
            widthText = str(int(widthCounts) if float(widthCounts).is_integer() else widthCounts)
        self.widthEdit.setPlaceholderText(widthText)
        self.stiffnessEdit.setPlaceholderText(str(self.hapticDevice.pseudoObject['stiffness']))
        self.dampingEdit.setPlaceholderText(str(self.hapticDevice.pseudoObject['damping']))

    def onStopAllShortcut(self) -> None:
        if self.isSerialConnected:
            self.stopAll()

    def stopAll(self) -> None:
        self.stopFingerButton.setChecked(True)
        self.stopFinger()
        self.stopPalmButton.setChecked(True)
        self.stopPalm()

    def homeFinger(self) -> None:
        self.messageToSend.emit("H0")

    def homePalm(self) -> None:
        self.messageToSend.emit("H1")

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
                self.updateControllerModeButtons()
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
        sender = self.sender()
        senderName = sender.objectName() if sender is not None else ""
        if senderName == "positionControlButton":
            controllerMode = "0"
            if motorIndex == "0":
                self.hapticDevice.fingerControlMode = 0
            elif motorIndex == "1":
                self.hapticDevice.palmControlMode = 0
        elif senderName == "velocityControlButton":
            controllerMode = "1"
            if motorIndex == "0":
                self.hapticDevice.fingerControlMode = 1
            elif motorIndex == "1":
                self.hapticDevice.palmControlMode = 1
        elif senderName == "forceControlButton":
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
        try:
            number = float(value)
        except ValueError:
            self.infoMessagesEdit.append(f"Invalid number: {value}")
            return
        generatorIndex = "0" if self.waveRadio1.isChecked() else "1"
        if makeUnitConversion:
            countToMm = st.COUNT_TO_MM if generatorIndex == "0" else st.COUNT_TO_MM2
            if countToMm == 0:
                self.infoMessagesEdit.append(
                    "<span style=\"font-size:8pt; font-weight:400; color:red;\" >"
                    "The mm conversion factor is 0. Set it in the settings or uncheck the mm box.</span>")
                return
            number = round(number / countToMm)
        elif number.is_integer():
            number = int(number)  # So that "5000" is not sent and shown as "5000.0"
        self.hapticDevice.waveformParams[int(generatorIndex)][paramType] = number
        self.setWaveStatusText()
        msg = f"W{paramType}{generatorIndex}{st.SERIAL_SEPERATOR_CH}{self.hapticDevice.waveformParams[int(generatorIndex)][paramType]}"
        # self.serialWorker.writeSerial(msg)
        self.messageToSend.emit(msg)

    @ pyqtSlot()
    def sendAllWaveParameters(self) -> None:
        self.sendWaveType()
        for edit, paramType in ((self.freqEdit, 'F'), (self.ampEdit, 'A'), (self.offsetEdit, 'O')):
            if edit.text() != "":
                self.sendWaveParameter(edit, paramType)

    @ pyqtSlot()
    def sendGain(self, textEdit: QTextEdit, gainType) -> None:
        value = textEdit.text()
        textEdit.setText("")
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
    def sendAllGains(self) -> None:
        for edit, gainType in ((self.KpEdit, "P"), (self.KiEdit, "I"), (self.KdEdit, "D")):
            if edit.text() != "":
                self.sendGain(edit, gainType)

    @ pyqtSlot()
    def sendPsuedoObject(self) -> None:
        widthText = self.widthEdit.text()
        stiffnessText = self.stiffnessEdit.text()
        dampingText = self.dampingEdit.text()
        self.widthEdit.setText("")
        self.stiffnessEdit.setText("")
        self.dampingEdit.setText("")

        width = self.hapticDevice.pseudoObject['width']
        stiffness = self.hapticDevice.pseudoObject['stiffness']
        damping = self.hapticDevice.pseudoObject['damping']

        if widthText != "":
            try:
                widthVal = float(widthText)
            except ValueError:
                self.infoMessagesEdit.append(f"Invalid number: {widthText}")
                return
            if self.unitCheckBox.isChecked():
                if st.COUNT_TO_MM == 0:
                    self.infoMessagesEdit.append(
                        "<span style=\"font-size:8pt; font-weight:400; color:red;\" >"
                        "The mm conversion factor is 0. Set it in the settings or uncheck the mm box.</span>")
                    return
                width = round(widthVal / st.COUNT_TO_MM)
            else:
                width = int(widthVal) if widthVal.is_integer() else widthVal

        if stiffnessText != "":
            try:
                stiffnessVal = float(stiffnessText)
            except ValueError:
                self.infoMessagesEdit.append(f"Invalid number: {stiffnessText}")
                return
            stiffness = int(stiffnessVal) if stiffnessVal.is_integer() else stiffnessVal

        if dampingText != "":
            try:
                dampingVal = float(dampingText)
            except ValueError:
                self.infoMessagesEdit.append(f"Invalid number: {dampingText}")
                return
            damping = int(dampingVal) if dampingVal.is_integer() else dampingVal

        self.hapticDevice.pseudoObject['width'] = width
        self.hapticDevice.pseudoObject['stiffness'] = stiffness
        self.hapticDevice.pseudoObject['damping'] = damping
        self.setObjectPlaceholders()

        msg = f"P{width}{st.SERIAL_SEPERATOR_CH}{stiffness}{st.SERIAL_SEPERATOR_CH}{damping}"
        # self.serialWorker.writeSerial(msg)
        self.messageToSend.emit(msg)

    def loadSettings(self, settings: QSettings) -> None:
        baudrate = settings.value("serial/baudrate", None)
        if baudrate is not None:
            self.baudrateEdit.setText(str(baudrate))
        unitsInMm = settings.value("serial/unitsInMm", None)
        if unitsInMm is not None:
            self.unitCheckBox.setChecked(str(unitsInMm).lower() == "true")

    def saveSettings(self, settings: QSettings) -> None:
        settings.setValue("serial/baudrate", self.baudrateEdit.text())
        settings.setValue("serial/unitsInMm", self.unitCheckBox.isChecked())

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

            # Blink the recording icon every second
            self.recordingIconTimer.start(1000)
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
        # Only one plot window at a time
        if self.animator is not None:
            self.animator.raise_()
            self.animator.activateWindow()
            return
        self.animator = QAnimator(hapticDevice=self.hapticDevice)
        # Connected to the methods of the animator, so the connections are removed when it is closed and deleted
        self.dataToPlot.connect(self.animator.updateData)
        self.newData.connect(self.animator.updateDataCount)
        self.rxRate.connect(self.animator.setInRate)
        self.animator.destroyed.connect(self.onAnimatorDestroyed)

    def onAnimatorDestroyed(self):
        self.animator = None

    def setDebug(self):
        if not self.debug:
            # self.serialWorker.writeSerial("B1")  # Set debug mode on
            self.messageToSend.emit("B1")  # Set debug mode on
        else:
            # self.serialWorker.writeSerial("B0")  # Set debug mode off
            self.messageToSend.emit("B0")  # Set debug mode off
