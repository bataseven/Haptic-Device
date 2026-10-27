import sys
import ctypes
import os
import tempfile
import traceback
from os.path import dirname, abspath
from UI.main_window import MainWindow
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication, QSplashScreen, QMessageBox
from PyQt5.QtCore import QEventLoop, QTimer, Qt, QLocale
from PyQt5 import QtGui
from UI import PyQt5_stylesheets as p5s

logPath = None
if sys.executable.endswith("pythonw.exe") or sys.stderr is None:
    # pythonw and the app built with PyInstaller have no console, so errors are written to a file
    logPath = os.path.join(tempfile.gettempdir(), "stderr-" + os.path.basename(sys.argv[0]))
    sys.stdout = open(os.devnull, "w")
    sys.stderr = open(logPath, "w")

if sys.platform == "win32":
    # Following 2 lines display the icon of the app on the taskbar.
    # https://stackoverflow.com/questions/1551605/how-to-set-applications-taskbar-icon-in-windows-7/1552105#1552105
    myappid = 'RML.HapticDevice.ControlInterface.v1'
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

shownErrors = set()


def showUnhandledException(excType, excValue, excTraceback):
    """
    Shows unhandled exceptions in a dialog. Without this, PyQt ends the app when a slot raises an exception.
    Each distinct error is shown once, so an error that repeats (e.g. in a timer) doesn't open dialog after dialog.
    """
    sys.__excepthook__(excType, excValue, excTraceback)
    if sys.stderr:
        sys.stderr.flush()
    error = f"{excType.__name__}: {excValue}"
    if QApplication.instance() is None or error in shownErrors:
        return
    shownErrors.add(error)
    text = "An unexpected error occurred. The app keeps running, but the last action may not have completed."
    if logPath:
        text += f"\n\nDetails are written to {logPath}"
    dialog = QMessageBox(QMessageBox.Critical, "Unexpected Error", text)
    dialog.setDetailedText("".join(traceback.format_exception(excType, excValue, excTraceback)))
    dialog.exec_()


sys.excepthook = showUnhandledException


def main():
    app = QApplication(sys.argv)

    # Numbers are typed with "." as the decimal point and without group separators, whatever the system language is.
    # The typed text is sent to the device as it is, and the device only reads this format.
    numberLocale = QLocale.c()
    numberLocale.setNumberOptions(QLocale.RejectGroupSeparator)
    QLocale.setDefault(numberLocale)

    try:
        style = "style_DarkOrange"
        styleSheet = p5s.load_stylesheet_pyqt5(style=style)
        app.setStyleSheet(styleSheet)
        font = QtGui.QFont("Helvetica", 9)
        # font.setStyleHint(QtGui.QFont.Times, QtGui.QFont.PreferAntialias)
        app.setFont(font)
    except Exception as e:
        print("Error loading stylesheet" + str(e))

    splashImagePath = dirname(abspath(__file__)) + "/UI/images/splash.png"
    pixmap = QPixmap(splashImagePath)
    splash = QSplashScreen(pixmap)
    splash.show()
    ui = MainWindow()


    ui.show()
    ui.centerOnScreen()
    splash.finish(ui)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
