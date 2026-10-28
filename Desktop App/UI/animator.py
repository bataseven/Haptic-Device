from collections import deque
from os.path import dirname, abspath
import pyqtgraph as pg
from PyQt5.QtWidgets import QPushButton, QLabel, QVBoxLayout, QGroupBox, QGridLayout, QHBoxLayout, QCheckBox, QWidget, QSlider
from PyQt5.QtCore import pyqtSlot, QTimer, Qt
from PyQt5.QtGui import QIcon

from .devices import HapticDevice
from .settings import serialSettings as st

pg.setConfigOptions(antialias=True)

FORCE_LABELS = {"Thumb Out", "Thumb In", "Index In", "Index Out", "Net Force"}
FINGER_KINEMATIC_LABELS = {"Finger Position", "Finger Velocity", "Finger Wave Output"}
PALM_KINEMATIC_LABELS = {"Palm Position", "Palm Velocity", "Palm Wave Output"}


class QAnimator(QWidget):
    """
    Live plot of the values received from the device. Each value has a check box to show or hide it.
    Drag to pan, scroll to zoom, right-click for more options such as exporting the plot.
    """

    def __init__(self, parent=None, hapticDevice: HapticDevice = None):
        super().__init__(parent)
        self.hapticDevice = hapticDevice
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
        self.forceMaxY = 1
        self.forceMinY = -1
        self.autoScaleY = True
        self.unitsInMm = False

        # Top plot: Kinematics & Waveforms
        self.plotWidget = pg.PlotWidget()
        self.plotItem = self.plotWidget.getPlotItem()
        self.plotItem.setLabel('bottom', 'Time [s]')
        self.plotItem.setLabel('left', 'Kinematics [Counts]')
        self.plotItem.showGrid(x=True, y=True, alpha=0.2)
        self.plotItem.disableAutoRange()  # The ranges are set in updatePlot()
        self.legend = self.plotItem.addLegend(offset=(10, 10))

        # Bottom plot: Force Sensors [N]
        self.forcePlotWidget = pg.PlotWidget()
        self.forcePlotItem = self.forcePlotWidget.getPlotItem()
        self.forcePlotItem.setLabel('bottom', 'Time [s]')
        self.forcePlotItem.setLabel('left', 'Forces [N]')
        self.forcePlotItem.showGrid(x=True, y=True, alpha=0.2)
        self.forcePlotItem.disableAutoRange()
        self.forcePlotItem.setXLink(self.plotItem)
        self.forceLegend = self.forcePlotItem.addLegend(offset=(10, 10))

        self.setMinimumSize(800, 600)

        interval = 50
        self.updateTimer = QTimer(self)
        self.updateTimer.timeout.connect(self.updatePlot)
        self.updateTimer.start(interval)

        self.pauseButton = QPushButton("Pause")
        self.pauseButton.setIcon(self.pauseIcon)
        self.pauseButton.pressed.connect(self.togglePause)

        self.clearButton = QPushButton("Clear")
        self.clearButton.setToolTip("Clear all plotted data points and reset axis ranges")
        self.clearButton.clicked.connect(self.clearPlot)

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

        self.unitCheckBox = QCheckBox("Units in mm")
        self.unitCheckBox.setChecked(False)
        self.unitCheckBox.setToolTip("Plot positions and velocities in mm and mm/s instead of encoder counts")
        self.unitCheckBox.stateChanged.connect(
            lambda state: self.setUnitsInMm(state == Qt.Checked))

        self.autoYButton = QCheckBox()
        self.autoYButton.setChecked(True)
        self.autoYButton.setText("Auto scale Y")
        self.autoYButton.stateChanged.connect(
            lambda state: self.setAutoScaleY(state == Qt.Checked))
        self.autoYButton.setToolTip(
            "Auto expand and contract Y axis if checked. Only expand if unchecked.")

        controlsLayout = QHBoxLayout()
        controlsLayout.addWidget(self.pauseButton)
        controlsLayout.addWidget(self.clearButton)
        controlsLayout.addWidget(self.updateTimerLabel)
        controlsLayout.addWidget(self.updateTimerSlider)
        controlsLayout.addWidget(self.unitCheckBox)
        controlsLayout.addWidget(self.autoYButton)

        self.buttonLayout = QGridLayout()
        self.buttonBox = QGroupBox()
        self.buttonBox.setLayout(self.buttonLayout)
        self.buttonColumns = 3

        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.plotWidget)
        mainLayout.addWidget(self.forcePlotWidget)
        mainLayout.addLayout(controlsLayout)
        mainLayout.addWidget(self.buttonBox)
        mainLayout.setStretch(0, 2)
        mainLayout.setStretch(1, 2)
        mainLayout.setStretch(3, 1)
        self.setLayout(mainLayout)

        # Label -> (points, curve). Points are [x, y] pairs of the last slideLimit values.
        self.data: dict = {}
        self.show()

    @staticmethod
    def isForceLabel(label: str) -> bool:
        return label in FORCE_LABELS

    def scaleValue(self, label: str, value: float) -> float:
        if not self.unitsInMm:
            return value
        if label in FINGER_KINEMATIC_LABELS:
            return value * st.COUNT_TO_MM
        if label in PALM_KINEMATIC_LABELS:
            return value * st.COUNT_TO_MM2
        return value

    def currentX(self) -> float:
        if self.hapticDevice is not None and self.hapticDevice.deviceTime > 0:
            return self.hapticDevice.deviceTime
        return float(self.count)

    def togglePause(self):
        if self.updateTimer.isActive():
            self.updateTimer.stop()
            self.pauseButton.setIcon(self.playIcon)
            self.pauseButton.setText("Resume")
        else:
            self.updateTimer.start()
            self.pauseButton.setIcon(self.pauseIcon)
            self.pauseButton.setText("Pause")

    def clearPlot(self):
        self.count = 0
        self.minY = -1
        self.maxY = 1
        self.forceMinY = -1
        self.forceMaxY = 1
        for points, curve in self.data.values():
            points.clear()
            curve.setData([], [])

    def setUnitsInMm(self, enabled: bool):
        self.unitsInMm = enabled
        self.minY = 0
        self.maxY = 0
        self.plotItem.setLabel('left', 'Kinematics [mm, mm/s]' if enabled else 'Kinematics [Counts]')
        self.updatePlot()

    def updatePlot(self):
        allXs = []
        for label, (points, curve) in self.data.items():
            if curve.isVisible() and len(points) > 0:
                xs = [x for x, _ in points]
                ys = [self.scaleValue(label, y) for _, y in points]
                curve.setData(xs, ys)
                allXs.extend(xs)

        usesDeviceTime = self.hapticDevice is not None and self.hapticDevice.deviceTime > 0
        bottomLabel = 'Time [s]' if usesDeviceTime else 'Data points'
        self.plotItem.setLabel('bottom', bottomLabel)
        self.forcePlotItem.setLabel('bottom', bottomLabel)

        if allXs:
            minX = min(allXs)
            maxX = max(allXs)
            if minX == maxX:
                maxX = minX + (1.0 if usesDeviceTime else self.slideLimit)
            self.plotItem.setXRange(minX, maxX, padding=0)
            self.forcePlotItem.setXRange(minX, maxX, padding=0)
        elif self.count < self.slideLimit:
            self.plotItem.setXRange(0, self.slideLimit, padding=0)
            self.forcePlotItem.setXRange(0, self.slideLimit, padding=0)
        else:
            self.plotItem.setXRange(self.count - self.slideLimit, self.count, padding=0)
            self.forcePlotItem.setXRange(self.count - self.slideLimit, self.count, padding=0)

        self.setLimits()
        self.plotItem.setTitle(f"{self.rate:.1f} Hz")

    @pyqtSlot(str, float)
    def updateData(self, label: str, value: float):
        x = self.currentX()
        # The data is not new, no need to add a button and line to the plot. Only add the value to deque.
        if label in self.data:
            self.data[label][0].append([x, float(value)])
            return

        color = pg.intColor(len(self.data), hues=9, values=1, maxValue=230)
        targetPlotItem = self.forcePlotItem if self.isForceLabel(label) else self.plotItem
        curve = targetPlotItem.plot(pen=pg.mkPen(color, width=2), name=label)
        self.data[label] = (deque([[x, float(value)]], maxlen=self.slideLimit), curve)

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
        self.updatePlotVisibility()

    def _updateSinglePlotLimits(self, plotItem, isForce: bool):
        maxY = 0
        minY = 0
        hasValues = False
        for label, (points, curve) in self.data.items():
            if self.isForceLabel(label) == isForce and curve.isVisible() and len(points) > 0:
                values = [self.scaleValue(label, y) for _, y in points]
                if not hasValues:
                    maxY = max(values)
                    minY = min(values)
                    hasValues = True
                else:
                    maxY = max(maxY, max(values))
                    minY = min(minY, min(values))

        if not hasValues:
            maxY = 0
            minY = 0

        if not self.autoScaleY:
            if isForce:
                self.forceMinY = min(self.forceMinY, minY)
                self.forceMaxY = max(self.forceMaxY, maxY)
                minY, maxY = self.forceMinY, self.forceMaxY
            else:
                self.minY = min(self.minY, minY)
                self.maxY = max(self.maxY, maxY)
                minY, maxY = self.minY, self.maxY

        def lerp(a, b, t): return a + (b - a) * t

        # Move towards the target range smoothly instead of jumping to it
        currentMin, currentMax = plotItem.viewRange()[1]
        if maxY == 0 and minY == 0:
            targetMin, targetMax = -1, 1
        elif minY == maxY:
            pad = abs(minY * 0.15) or 1.0
            targetMin, targetMax = minY - pad, maxY + pad
        else:
            spanPad = (maxY - minY) * 0.15
            targetMin, targetMax = minY - spanPad, maxY + spanPad
        plotItem.setYRange(lerp(currentMin, targetMin, 0.85), lerp(currentMax, targetMax, 0.85), padding=0)

    def setLimits(self):
        self._updateSinglePlotLimits(self.plotItem, isForce=False)
        self._updateSinglePlotLimits(self.forcePlotItem, isForce=True)

    def setAutoScaleY(self, state: bool):
        self.autoScaleY = state
        self.minY = 0
        self.maxY = 0
        self.forceMinY = 0
        self.forceMaxY = 0

    def updatePlotVisibility(self):
        hasVisibleKinematics = any(
            not self.isForceLabel(lbl) and curve.isVisible() for lbl, (_, curve) in self.data.items()
        )
        hasVisibleForces = any(
            self.isForceLabel(lbl) and curve.isVisible() for lbl, (_, curve) in self.data.items()
        )
        if not hasVisibleKinematics and not hasVisibleForces:
            self.plotWidget.setVisible(True)
            self.forcePlotWidget.setVisible(True)
        else:
            self.plotWidget.setVisible(hasVisibleKinematics)
            self.forcePlotWidget.setVisible(hasVisibleForces)

    def setLineVisibility(self, visible: bool, label: str):
        if label not in self.data:
            return
        curve = self.data[label][1]
        curve.setVisible(visible)
        targetLegend = self.forceLegend if self.isForceLabel(label) else self.legend
        # Only list the visible lines in the legend
        if visible:
            targetLegend.addItem(curve, label)
        else:
            targetLegend.removeItem(curve)
        self.updatePlotVisibility()

    @pyqtSlot()
    def updateDataCount(self):
        self.count += 1

    @pyqtSlot(float)
    def setInRate(self, rate):
        self.rate = rate

    def closeEvent(self, event):
        self.updateTimer.stop()
        super().closeEvent(event)
