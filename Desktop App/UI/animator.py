from collections import deque
import random
import matplotlib.pyplot as plt
from matplotlib import rcParams
import matplotx
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavigationToolbar
from os.path import dirname, abspath
from PyQt5.QtWidgets import QPushButton, QLabel, QVBoxLayout, QGroupBox, QGridLayout, QHBoxLayout, QCheckBox, QWidget, QSlider, QMenuBar, QDialog, QMainWindow
from PyQt5.QtCore import pyqtSlot,  QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QIcon


rcParams.update({'figure.autolayout': True})
plt.style.use(matplotx.styles.dracula)

class QAnimator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.setWindowTitle("Animator")
        try:
            self.icon = QIcon(dirname(abspath(__file__)) +
                              "/images/plotter.png")
            self.setWindowIcon(self.icon)
        except Exception as e:
            print(e)

        try:
            self.pauseIcon = QIcon(
                dirname(abspath(__file__)) + "/images/pause.png")
        except Exception as e:
            print(e)

        try:
            self.playIcon = QIcon(
                dirname(abspath(__file__)) + "/images/play.png")
        except Exception as e:
            print(e)

        self.rate = 0
        self.count = 0
        self.slideLimit = 125
        self.maxY = 1
        self.minY = -1
        self.autoScaleY = True

        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self, False)
        self.setMinimumSize(800, 600)
        self.axes = self.figure.add_subplot()
        self.axes.set_xlabel('Data points')
        self.axes.set_ylabel('Values')

        interval = 250
        self.updateTimer = QTimer()
        self.updateTimer.timeout.connect(self.updatePlot)
        self.updateTimer.start(interval)

        self.pauseButton = QPushButton()
        self.pauseButton.setIcon(self.pauseIcon)
        self.pauseButton.pressed.connect(lambda: self.pauseButton.setIcon(
            self.playIcon) if self.updateTimer.isActive() else self.pauseButton.setIcon(self.pauseIcon))
        self.pauseButton.pressed.connect(lambda: self.updateTimer.stop(
        ) if self.updateTimer.isActive() else self.updateTimer.start())

        self.updateTimerLabel = QLabel()
        self.updateTimerLabel.setText(f"Update plot every {interval} ms")
        self.updateTimerLabel.setToolTip(
            "Higher the value better the app performance")

        self.updateTimerSlider = QSlider(Qt.Horizontal)
        self.updateTimerSlider.setMinimum(10)
        self.updateTimerSlider.setMaximum(1000)
        self.updateTimerSlider.setValue(interval)
        self.updateTimerSlider.setTickInterval(10)
        self.updateTimerSlider.setSingleStep(10)
        self.updateTimerSlider.setToolTip(
            "Higher the value better the app performance")
        self.updateTimerSlider.setTickPosition(QSlider.TicksBelow)
        self.updateTimerSlider.valueChanged.connect(
            lambda interval: self.updateTimer.setInterval(interval))
        self.updateTimerSlider.valueChanged.connect(
            lambda interval: self.updateTimerLabel.setText(f"Update plot every {interval} ms"))

        self.autoYButton = QCheckBox()
        self.autoYButton.setChecked(True)
        self.autoYButton.setText("Auto scale Y")
        self.autoYButton.stateChanged.connect(
            lambda state: self.setAutoScaleY(state))
        self.autoYButton.setToolTip(
            "Auto expand and contract Y axis if checked. Only expand if unchecked.")

        layout = QHBoxLayout()
        layout.addWidget(self.pauseButton)
        layout.addWidget(self.updateTimerLabel)
        layout.addWidget(self.updateTimerSlider)
        layout.addWidget(self.autoYButton)

        self.buttonLayout = QGridLayout()
        self.buttonBox = QGroupBox()

        self.buttonBox.setLayout(self.buttonLayout)
        self.buttonColumns = 3

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.toolbar)
        self.layout.addWidget(self.canvas)
        self.layout.addLayout(layout)
        self.layout.addWidget(self.buttonBox)
        self.layout.setStretch(1, 4)
        self.layout.setStretch(3, 1)
        self.setLayout(self.layout)

        self.data: dict = {}
        self.show()

    def updatePlot(self):
        for key in self.data.keys():
            if self.data[key][1].get_visible():
                self.data[key][1].set_data(*zip(*self.data[key][0]))

        if self.count < self.slideLimit:
            self.axes.set_xlim(0, self.slideLimit)
        else:
            self.axes.set_xlim(
                self.count - self.slideLimit, self.count)
        self.setLimits()
        self.axes.set_title(f"{self.rate:.1f} Hz")

        self.canvas.draw_idle()

    def updateData(self, label: str, value: float):
        # The data is not new, no need to add a button and line to the plot. Only add the value to deque.
        if(label in self.data.keys()):
            self.data[label][0].append([self.count, float(value)])
        else:
            checkBox = QCheckBox()
            checkBox.setChecked(True)
            checkBox.setText(label)
            checkBox.stateChanged.connect(lambda state: self.setLineVisibility(
                state == Qt.Checked, label))
            checkBox.stateChanged.connect(lambda _: self.setLegends())
            buttonCount = self.buttonBox.layout().count()
            rows = buttonCount // self.buttonColumns
            columns = buttonCount % self.buttonColumns
            self.buttonBox.layout().addWidget(checkBox, rows, columns)

            dataPoints = deque([[self.count, float(value)]],
                               maxlen=self.slideLimit)
            self.data[label] = []
            self.data[label].append(dataPoints)
            lineColor = (random.random() * 0.8, random.random() *
                         0.8, random.random() * 0.8)
            line, = self.axes.plot(*zip(*dataPoints), color=lineColor)
            # Get the color of the line and set it to the checkbox
            checkBox.setStyleSheet(
                f"QCheckBox {{ color: rgb({lineColor[0]*255}, {lineColor[1]*255}, {lineColor[2]*255}) }} QCheckBox::indicator::checked {{ background-color: rgb({lineColor[0]*255}, {lineColor[1]*255}, {lineColor[2]*255}) }}")
            line.set_label(label)
            self.data[label].append(line)
            self.setLegends()

    def setLimits(self):
        maxY = 0
        minY = 0
        for key in self.data.keys():
            if self.data[key][1].get_visible():
                data = self.data[key][1].get_ydata()
                if max(data) > maxY:
                    maxY = max(data)
                if min(data) < minY:
                    minY = min(data)

        if not self.autoScaleY:
            self.minY = min(self.minY, minY)
            self.maxY = max(self.maxY, maxY)

        def lerp(a, b, t): return a + (b - a) * t

        if maxY == 0 and minY == 0:
            lerpedMin = lerp(self.axes.get_ylim()[0], -1, 0.85)
            lerpedMax = lerp(self.axes.get_ylim()[1], 1, 0.85)
            self.axes.set_ylim(lerpedMin, lerpedMax)

        elif self.autoScaleY:
            lerpedMin = lerp(self.axes.get_ylim()[
                             0], minY - abs(minY * 0.15), 0.85)
            lerpedMax = lerp(self.axes.get_ylim()[
                             1], maxY + abs(maxY * 0.15), 0.85)
            self.axes.set_ylim(lerpedMin, lerpedMax)
        else:
            lerpedMin = lerp(self.axes.get_ylim()[
                             0], self.minY - abs(self.minY * 0.15), 0.85)
            lerpedMax = lerp(self.axes.get_ylim()[
                             1], self.maxY + abs(self.maxY * 0.15), 0.85)
            self.axes.set_ylim(lerpedMin, lerpedMax)

    def setAutoScaleY(self, state):
        self.canvas.draw_idle()
        self.autoScaleY = state
        self.minY = 0
        self.maxY = 0

    def setLegends(self):
        # Do not show the legend if the line is not visible.
        linesToPlot = [self.data[key][1]
                       for key in self.data.keys() if self.data[key][1].get_visible()]
        labelsInLegend = [key for key in self.data.keys(
        ) if self.data[key][1].get_visible()]

        self.axes.legend(linesToPlot, labelsInLegend, loc='upper left')

    def setLineVisibility(self, clicked: bool, label: str):
        if(label in self.data.keys()):
            self.data[label][1].set_visible(clicked)
            self.axes.relim()
            self.axes.autoscale_view()
            self.canvas.draw_idle()

    def updateDataCount(self):
        self.count += 1

    def setInRate(self, rate):
        self.rate = rate

    def showThemeDialog(self):
        dialog = QDialog()
        dialog.exec_()
