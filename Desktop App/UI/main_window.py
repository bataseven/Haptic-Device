import time
from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QDesktopWidget, QMessageBox, QLabel, QPushButton, QDialogButtonBox, QApplication
from UI.control_page import ControlPage
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt
from PyQt5 import sip
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
        self.controlPage.serialHandler.serialConnected.connect(self.recordDataAction.setEnabled)

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
        self.recordDataAction = self.fileMenu.addAction("Record Data...", self.showRecordDataDialog)
        self.recordDataAction.setEnabled(False)

        self.helpMenu.addAction("About...", self.about)
        self.helpMenu.addAction(
            "Show Serial Commands...", self.showSerialCommands)

    @staticmethod
    def isOpen(window: QWidget) -> bool:
        # Windows with WA_DeleteOnClose are deleted when closed, and calling isVisible() on them would raise an error
        return not sip.isdeleted(window) and window.isVisible()

    def showSettingsDialog(self):
        if self.isOpen(self.settingsWindow):
            self.settingsWindow.raise_()
            self.settingsWindow.activateWindow()
            return
        serialHandler = self.controlPage.serialHandler
        self.settingsWindow = SettingsWindow(parent=None, hapticDevice=self.hapticDevice)
        self.settingsWindow.messageToSend.connect(serialHandler.messageToSend)
        # Connected to the methods of the window, so the connections are removed when the window is deleted
        serialHandler.serialConnected.connect(self.settingsWindow.onSerialConnected)
        serialHandler.newLimits.connect(self.settingsWindow.setLimitTexts)
        self.settingsWindow.onSerialConnected(serialHandler.isSerialConnected)

        self.settingsWindow.show()
        self.centerWidgetOnScreen(self.settingsWindow)

    def showSerialCommands(self):
        self.serialCommandsWindow = SerialCommandsWindow(self.parent())
        self.serialCommandsWindow.show()
        self.centerWidgetOnScreen(self.serialCommandsWindow)

    def showRecordDataDialog(self):
        if self.isOpen(self.dataRecorder):
            self.dataRecorder.raise_()
            self.dataRecorder.activateWindow()
            return
        serialHandler = self.controlPage.serialHandler
        self.dataRecorder = DataRecorder(
            parent=self.parent(), hapticDevice=self.hapticDevice, portName=serialHandler.portName)
        self.dataRecorder.show()
        # Connected to the methods of the recorder, so the connections are removed when it is deleted
        serialHandler.serialConnected.connect(self.dataRecorder.onSerialConnected)
        serialHandler.newData.connect(self.dataRecorder.appendToFile)
        self.centerWidgetOnScreen(self.dataRecorder)
        # Closing the recorder stops the recording, which also resets the recording icon
        self.dataRecorder.recordingInProgress.connect(serialHandler.recording)

    def closeEvent(self, event):
        # Disconnect properly, so the device disarms its host timeout and keeps working with the next program
        self.controlPage.serialHandler.serialWorker.disconnectSerial()
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