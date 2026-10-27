from collections import deque
from os.path import dirname, abspath
import pyqtgraph as pg
from PyQt5.QtWidgets import QPushButton, QLabel, QVBoxLayout, QGroupBox, QGridLayout, QHBoxLayout, QCheckBox, QWidget, QSlider
from PyQt5.QtCore import pyqtSlot, QTimer, Qt
from PyQt5.QtGui import QIcon

pg.setConfigOptions(antialias=True)


class QAnimator(QWidget):
    """
    Live plot of the values received from the device. Each value has a check box to show or hide it.
    Drag to pan, scroll to zoom, right-click for more options such as exporting the plot.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.setWindowTitle("Animator")
        self.icon = QIcon(dirname(abspath(__file__)) + "/images/plotter.png")
        self.setWindowIcon(self.icon)
        self.pauseIcon = QIcon(dirname(abspath(__file__)) + "/images/pause.png")
        self.playIcon = QIcon(dirname(abspath(__file__)) + "/images/play.png")

        self.rate = 0
        self.count = 0
        self.slideLimit = 125
        self.maxY = 1
        self.minY = -1
        self.autoScaleY = True

        self.plotWidget = pg.PlotWidget()
        self.plotItem = self.plotWidget.getPlotItem()
        self.plotItem.setLabel('bottom', 'Data points')
        self.plotItem.setLabel('left', 'Values')
        self.plotItem.showGrid(x=True, y=True, alpha=0.2)
        self.plotItem.disableAutoRange()  # The ranges are set in updatePlot()
        self.legend = self.plotItem.addLegend(offset=(10, 10))
        self.setMinimumSize(800, 600)

        interval = 50
        self.updateTimer = QTimer(self)
        self.updateTimer.timeout.connect(self.updatePlot)
        self.updateTimer.start(interval)

        self.pauseButton = QPushButton()
        self.pauseButton.setIcon(self.pauseIcon)
        self.pauseButton.pressed.connect(self.togglePause)

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
            lambda state: self.setAutoScaleY(state == Qt.Checked))
        self.autoYButton.setToolTip(
            "Auto expand and contract Y axis if checked. Only expand if unchecked.")

        controlsLayout = QHBoxLayout()
        controlsLayout.addWidget(self.pauseButton)
        controlsLayout.addWidget(self.updateTimerLabel)
        controlsLayout.addWidget(self.updateTimerSlider)
        controlsLayout.addWidget(self.autoYButton)

        self.buttonLayout = QGridLayout()
        self.buttonBox = QGroupBox()
        self.buttonBox.setLayout(self.buttonLayout)
        self.buttonColumns = 3

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.plotWidget)
        mainLayout.addLayout(controlsLayout)
        mainLayout.addWidget(self.buttonBox)
        mainLayout.setStretch(0, 4)
        mainLayout.setStretch(2, 1)
        self.setLayout(mainLayout)

        # Label -> (points, curve). Points are [x, y] pairs of the last slideLimit values.
        self.data: dict = {}
        self.show()

    def togglePause(self):
        if self.updateTimer.isActive():
            self.updateTimer.stop()
            self.pauseButton.setIcon(self.playIcon)
        else:
            self.updateTimer.start()
            self.pauseButton.setIcon(self.pauseIcon)

    def updatePlot(self):
        for points, curve in self.data.values():
            if curve.isVisible():
                xs, ys = zip(*points)
                curve.setData(xs, ys)

        if self.count < self.slideLimit:
            self.plotItem.setXRange(0, self.slideLimit, padding=0)
        else:
            self.plotItem.setXRange(self.count - self.slideLimit, self.count, padding=0)
        self.setLimits()
        self.plotItem.setTitle(f"{self.rate:.1f} Hz")

    @pyqtSlot(str, float)
    def updateData(self, label: str, value: float):
        # The data is not new, no need to add a button and line to the plot. Only add the value to deque.
        if label in self.data:
            self.data[label][0].append([self.count, float(value)])
            return

        color = pg.intColor(len(self.data), hues=9, values=1, maxValue=230)
        curve = self.plotItem.plot(pen=pg.mkPen(color, width=2), name=label)
        self.data[label] = (deque([[self.count, float(value)]], maxlen=self.slideLimit), curve)

        checkBox = QCheckBox()
        checkBox.setChecked(True)
        checkBox.setText(label)
        rgb = f"rgb({color.red()}, {color.green()}, {color.blue()})"
        checkBox.setStyleSheet(
            f"QCheckBox {{ color: {rgb} }} QCheckBox::indicator::checked {{ background-color: {rgb} }}")
        checkBox.stateChanged.connect(lambda state: self.setLineVisibility(state == Qt.Checked, label))
        buttonCount = self.buttonBox.layout().count()
        rows = buttonCount // self.buttonColumns
        columns = buttonCount % self.buttonColumns
        self.buttonBox.layout().addWidget(checkBox, rows, columns)

    def setLimits(self):
        maxY = 0
        minY = 0
        for points, curve in self.data.values():
            if curve.isVisible():
                values = [y for _, y in points]
                maxY = max(maxY, max(values))
                minY = min(minY, min(values))

        if not self.autoScaleY:
            self.minY = min(self.minY, minY)
            self.maxY = max(self.maxY, maxY)

        def lerp(a, b, t): return a + (b - a) * t

        # Move towards the target range smoothly instead of jumping to it
        currentMin, currentMax = self.plotItem.viewRange()[1]
        if maxY == 0 and minY == 0:
            targetMin, targetMax = -1, 1
        elif self.autoScaleY:
            targetMin, targetMax = minY - abs(minY * 0.15), maxY + abs(maxY * 0.15)
        else:
            targetMin, targetMax = self.minY - abs(self.minY * 0.15), self.maxY + abs(self.maxY * 0.15)
        self.plotItem.setYRange(lerp(currentMin, targetMin, 0.85), lerp(currentMax, targetMax, 0.85), padding=0)

    def setAutoScaleY(self, state: bool):
        self.autoScaleY = state
        self.minY = 0
        self.maxY = 0

    def setLineVisibility(self, visible: bool, label: str):
        if label not in self.data:
            return
        curve = self.data[label][1]
        curve.setVisible(visible)
        # Only list the visible lines in the legend
        if visible:
            self.legend.addItem(curve, label)
        else:
            self.legend.removeItem(curve)

    @pyqtSlot()
    def updateDataCount(self):
        self.count += 1

    @pyqtSlot(float)
    def setInRate(self, rate):
        self.rate = rate
