import time
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QDesktopWidget, QMessageBox, QLabel, QPushButton, QDialogButtonBox, QApplication
from UI.control_page import ControlPage
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt
from os.path import dirname, abspath
from UI.serial_commands_window import SerialCommandsWindow
from UI.settings_window import SettingsWindow
from UI.data_recorder import DataRecorder
from .devices import HapticDevice
from .settings import serialSettings as st

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Haptic Device Interface")
        self.hapticDevice = HapticDevice()
        self.settingsWindow = QWidget()
        self.serialCommandsWindow = QWidget()
        self.dataRecorder = QWidget()
        self.setupUi()

        # Activate the record data menu action when the serial is connected and deactivate it when the serial is disconnected
        self.controlPage.serialHandler.serialConnected.connect(lambda connected: self.fileMenu.actions()[2].setEnabled(connected))

    def setupUi(self):
        iconPath = dirname(abspath(__file__)) + "/images/assembled.png"
        self.icon = QIcon(iconPath)
        self.setWindowIcon(self.icon)
        self.addMenuBar()
        self.controlPage = ControlPage(
            parent=self, hapticDevice=self.hapticDevice)
        self.setCentralWidget(self.controlPage)

    def centerOnScreen(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def addMenuBar(self):
        self.menuBar = self.menuBar()
        self.fileMenu = self.menuBar.addMenu("&File")
        self.helpMenu = self.menuBar.addMenu("&Help")

        self.fileMenu.addAction("E&xit", self.close)
        self.fileMenu.addAction("Settings...", self.showSettingsDialog)
        self.fileMenu.addAction("Record Data...", self.showRecordDataDialog).setEnabled(False)

        self.helpMenu.addAction("About...", self.about)
        self.helpMenu.addAction(
            "Show Serial Commands...", self.showSerialCommands)

    def showSettingsDialog(self):
        self.settingsWindow = SettingsWindow(parent=None, hapticDevice=self.hapticDevice)
        # Check if there is another instance of the settings window and if so, destroy it
        self.controlPage.serialHandler.serialConnected.connect(
            lambda _: self.settingsWindow.portLabel.setText(st.PORT))
        self.controlPage.serialHandler.serialConnected.connect(
            lambda _: self.settingsWindow.baudrateLabel.setText(str(st.BAUDRATE)))
        self.settingsWindow.messageToSend.connect(lambda msg: self.controlPage.serialHandler.messageToSend.emit(msg))

        # Enable the buttons if the serial is alrady connected when the settings window is opened
        self.settingsWindow.fingerSoftSendButton.setEnabled(self.controlPage.serialHandler.isSerialConnected)
        self.settingsWindow.fingerHardSendButton.setEnabled(self.controlPage.serialHandler.isSerialConnected)
        self.settingsWindow.palmSoftSendButton.setEnabled(self.controlPage.serialHandler.isSerialConnected)
        self.settingsWindow.palmHardSendButton.setEnabled(self.controlPage.serialHandler.isSerialConnected)

        self.controlPage.serialHandler.serialConnected.connect( lambda connected: self.settingsWindow.fingerSoftSendButton.setEnabled(connected))
        self.controlPage.serialHandler.serialConnected.connect( lambda connected: self.settingsWindow.fingerHardSendButton.setEnabled(connected))
        self.controlPage.serialHandler.serialConnected.connect( lambda connected: self.settingsWindow.palmSoftSendButton.setEnabled(connected))
        self.controlPage.serialHandler.serialConnected.connect( lambda connected: self.settingsWindow.palmHardSendButton.setEnabled(connected))
        self.controlPage.serialHandler.newLimits.connect(self.settingsWindow.setLimitTexts)

        self.settingsWindow.destroyed.connect(lambda: self.controlPage.serialHandler.serialConnected.disconnect())
        
        self.settingsWindow.show()
        self.centerWidgetOnScreen(self.settingsWindow)

    def showSerialCommands(self):
        self.serialCommandsWindow = SerialCommandsWindow(self.parent())
        self.serialCommandsWindow.show()
        self.centerWidgetOnScreen(self.serialCommandsWindow)

    def showRecordDataDialog(self):       
        self.dataRecorder = DataRecorder(
            parent=self.parent(), hapticDevice=self.hapticDevice, portName=self.controlPage.serialHandler.portName)
        self.dataRecorder.show()
        self.controlPage.serialHandler.serialConnected.connect(
            lambda connected: self.dataRecorder.stopRecording() if not connected else None)
        self.controlPage.serialHandler.serialConnected.connect(
            lambda connected: self.dataRecorder.recordButton.setEnabled(connected))
        self.controlPage.serialHandler.newData.connect(
            self.dataRecorder.appendToFile)
        self.centerWidgetOnScreen(self.dataRecorder)
        self.dataRecorder.recordingInProgress.connect(lambda recording: self.controlPage.serialHandler.recording(recording))
        self.dataRecorder.destroyed.connect(lambda: self.controlPage.serialHandler.recording(False))

    def closeEvent(self, event):
        for window in QApplication.topLevelWidgets():
            window.close()

    def centerWidgetOnScreen(self, widget: QWidget):
        """Centers the window on the screen."""
        resolution = QDesktopWidget().screenGeometry()
        widget.move(int((resolution.width() / 2) - (widget.frameSize().width() / 2)),
                    int((resolution.height() / 2) - (widget.frameSize().height() / 2)))
        

    def about(self):
        # Show a message dialog
        dialog = QMessageBox()
        dialog.setWindowTitle("About")
        dialog.setWindowIcon(self.icon)
        dialog.setIcon(QMessageBox.Information)
        dialog.findChildren(QLabel)[1].setAlignment(Qt.AlignJustify)
        # Create an html text to show in the dialog
        text = "<html><head/><body>\
        <p><span style=\" font-size:9pt; font-weight:normal; text-align=justify;\">This desktop app was created to communicate with the \
        <a href=\"https://bataseven.github.io/index.html#projects\">handheld haptic device</a> through serial port. Select the port and the baudrate of the device to \
        establish connection with the device. The program sends commands in a predetermined format to control the device. \
        These commands are listed under &ldquo;Help&rdquo; menu.</span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">Version Number: 1.0.0</span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\">Author: <em>Berke Ataseven</em> (bataseven15@ku.edu.tr)</span></p>\
        <p><span style=\" font-size:9pt; font-weight:normal;\"><em>© Koc University, Robotics and Mechatronics Laboratory</em></span></p>\
        </body></html>"
        dialog.setText(text)
        dialog.setStandardButtons(QMessageBox.Ok)
        dialog.exec_()