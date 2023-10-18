import time
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QDesktopWidget, QMessageBox, QLabel, QPushButton, QDialogButtonBox
from UI.control_page import ControlPage
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt
from os.path import dirname, abspath
from PyQt5.uic import loadUi  # type: ignore


class SerialCommandsWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Serial Commands")
        loadUi(dirname(abspath(__file__)) + '/serial_commands_window.ui', self)

        self.label1: QLabel
        self.label2: QLabel
        self.label3: QLabel

        self.setupUi()

    def setupUi(self):
        # Set window icon to the app icon.
        self.setWindowIcon(
            QIcon(dirname(abspath(__file__)) + "/images/assembled.png"))

        text1 = "<html><head/><body>\
        <p><span style=\" font-size:9pt; font-weight:normal;\"> For both the haptic device and the computer, received and sent message packets begin \
        (&lt;) and end (&gt;) with specific characters. If a message packet contains multiple fields, they are seperated with (#) character.\
        This type of encoding enables easy parsing of the message packets. If a received message complies with the format, it is displayed in the <em>Serial Messages</em> window with a green text. If not, it is displayed with a red text. \
        Below are the message types associated with each command:</span></p>\
        </body></html>"
        text1 += "<html><head/><body>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">\
        </body></html>"

        text2 = "<html><head/><body>\
        <p><span style=\" font-size:9pt; font-weight:normal;\"> \
        <p><span style=\" font-size:9pt; font-weight:bold;\">While receiving messages from the haptic device:</span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>Haptic Device Status</u><br><b>Message Type:</b> H + MotorIndex + EncoderPos + EncoderSpeed <br><b>Example:</b> <em>H10#0</em></span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>Sensor Force Values</u><br><b>Message Type:</b> F + Force1 + # + Force2 + # + Force3 + # + Force4 <br><b>Example:</b> <em>F0.5#0.5#0.5#0.5</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>Net Finger Force</u><br><b>Message Type:</b> N + NetFingerForce<br><b>Example:</b> <em>N3.2</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>Desired Finger Force</u><br><b>Message Type:</b> D + DesiredForce <br><b>Example:</b> <em>D4.5</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>IMU Euler Angles</u><br><b>Message Type:</b> E + Yaw + # + Roll + # + Pitch <br><b>Example:</b> <em>E10.5#3.5#128.3</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>PID Gains of Position Velocity and Force Controllers </u><br><b>Message Type:</b> K + MotorIndex + Kp1 + # + Ki1 + # + Kd1 + # + Kp2 + # + Ki2 + # + Kd2 + # + Kp3 + # + Ki3 + # + Kd3<br><b>Example:</b> <em>K01.00#0.00#6.00#0.05#0.01#0.00#1000.00#5.00#100.00</em></span></p>\
        </body></html>"

        text3 = "<html><head/><body>\
        <p><span style=\" font-size:9pt; font-weight:normal;\"> \
        <p><span style=\" font-size:9pt; font-weight:bold;\">While sending messages to the haptic device:</span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>To specify which program is connected to the haptic device over the serial port</u><br><b>Message Type:</b> C + Program <br><b>Example:</b> <em>C0</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>To stop the motors</u><br><b>Message Type:</b> S + MotorIndex + Stop(0 or 1) <br><b>Example:</b> <em>S01</em></span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>To change PID gains</u><br><b>Message Type:</b> K + P or I or D + MotorIndex + # + Gain <br><b>Example:</b> <em>KP1#0.5</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>To change the waveform parameters</u><br><b>Message Type:</b> W + T or F or A or O + GeneratorIndex + # + Value <br><b>Example:</b> <em>WF1#0.5</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>To change the controller mode</u><br><b>Message Type:</b> O + MotorIndex + ControllerMode <br><b>Example:</b> <em>O10</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>To set a virtual object</u><br><b>Message Type:</b> P + NominalWidth + # + Stiffness + # + Damping<br><b>Example:</b> <em>P50000#2.5#0.1</em> </span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">• <u>To set serial debug mode</u><br><b>Message Type:</b> B + DebugMode <br><b>Example:</b> <em>B0</em> </span></p>\
        </body></html>"

        self.label1.setAlignment(Qt.AlignJustify)
        self.label1.setWordWrap(True)
        self.label1.setText(text1)

        self.label2.setText(text2)
        self.label2.setAlignment(Qt.AlignLeft)

        self.label3.setText(text3)
        self.label3.setAlignment(Qt.AlignLeft)
