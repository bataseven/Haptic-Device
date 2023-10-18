import sys
import ctypes
import os
from os.path import dirname, abspath
from UI.main_window import MainWindow
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication, QSplashScreen
from PyQt5.QtCore import QEventLoop, QTimer, Qt
from PyQt5 import QtGui
from UI import PyQt5_stylesheets as p5s

if sys.executable.endswith("pythonw.exe"):
    sys.stdout = open(os.devnull, "w")
    sys.stderr = open(os.path.join(os.getenv("TEMP"),
                      "stderr-"+os.path.basename(sys.argv[0])), "w")

# Following 3 line displays the icon of the app on the taskbar.
# https://stackoverflow.com/questions/1551605/how-to-set-applications-taskbar-icon-in-windows-7/1552105#1552105
myappid = 'RML.HapticDevice.ControlInterface.v1'
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)


def main():
    app = QApplication(sys.argv)

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
