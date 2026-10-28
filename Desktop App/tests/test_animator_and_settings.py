import os
import tempfile
import unittest

import helpers  # noqa: F401  Creates the Qt application
from PyQt5.QtCore import QSettings
from UI.animator import QAnimator
from UI.data_recorder import DataRecorder
from UI.devices import HapticDevice
from UI.serial_handler import SerialHandler
from UI.settings import serialSettings as st
from UI.settings_window import SettingsWindow


class AnimatorTest(unittest.TestCase):
    def setUp(self):
        self.device = HapticDevice()
        self.animator = QAnimator(hapticDevice=self.device)
        self.addCleanup(self.animator.close)
        oldFactors = (st.COUNT_TO_MM, st.COUNT_TO_MM2)
        self.addCleanup(setattr, st, "COUNT_TO_MM", oldFactors[0])
        self.addCleanup(setattr, st, "COUNT_TO_MM2", oldFactors[1])

    def test_separates_kinematics_and_forces_into_two_plots(self):
        self.device.setDeviceTime(1_000_000)
        self.animator.updateData("Finger Position", 10000)
        self.animator.updateData("Net Force", 2.5)
        self.animator.updatePlot()

        self.assertIn(self.animator.data["Finger Position"][1], self.animator.plotItem.listDataItems())
        self.assertIn(self.animator.data["Net Force"][1], self.animator.forcePlotItem.listDataItems())
        self.assertEqual(self.animator.data["Finger Position"][0][0], [1.0, 10000.0])

    def test_units_in_mm_scales_kinematics_but_not_forces(self):
        st.COUNT_TO_MM = 0.01
        self.animator.updateData("Finger Position", 500)
        self.animator.updateData("Net Force", 3.0)
        self.animator.unitCheckBox.setChecked(True)
        self.assertEqual(self.animator.scaleValue("Finger Position", 500), 5.0)
        self.assertEqual(self.animator.scaleValue("Net Force", 3.0), 3.0)

    def test_pause_and_clear_reset_plot_data(self):
        self.animator.updateData("Finger Position", 500)
        self.animator.togglePause()
        self.assertFalse(self.animator.updateTimer.isActive())
        self.assertEqual(self.animator.pauseButton.text(), "Resume")

        self.animator.clearPlot()
        self.assertEqual(len(self.animator.data["Finger Position"][0]), 0)


class SettingsAndPersistenceTest(unittest.TestCase):
    def test_settings_window_displays_mm_equivalents_for_limits(self):
        oldFactors = (st.COUNT_TO_MM, st.COUNT_TO_MM2)
        self.addCleanup(setattr, st, "COUNT_TO_MM", oldFactors[0])
        self.addCleanup(setattr, st, "COUNT_TO_MM2", oldFactors[1])
        st.COUNT_TO_MM = 0.001
        st.COUNT_TO_MM2 = 0.01

        device = HapticDevice()
        device.fingerSoftLimits = [0, 160000]
        device.palmSoftLimits = [100, 2000]
        window = SettingsWindow(device)
        self.addCleanup(window.close)

        self.assertEqual(window.fingerSoftMmLabel.text(), "(0.00 to 160.00 mm)")
        self.assertEqual(window.palmSoftMmLabel.text(), "(1.00 to 20.00 mm)")

    def test_qsettings_persistence_for_serial_handler_and_recorder(self):
        fd, path = tempfile.mkstemp(suffix=".ini")
        os.close(fd)
        self.addCleanup(os.remove, path)
        settings = QSettings(path, QSettings.IniFormat)

        device = HapticDevice()
        handler = SerialHandler(device)
        handler.baudrateEdit.setText("230400")
        handler.unitCheckBox.setChecked(True)
        handler.saveSettings(settings)

        recorder = DataRecorder(device, "COM3")
        recorder.saveDurationEdit.setText("120")
        recorder.filePrefixEdit.setText("TrialA")
        recorder.continuousCheckBox.setChecked(True)
        recorder.virtualObjectCheckBox.setChecked(False)
        recorder.saveSettings(settings)

        handler2 = SerialHandler(device)
        handler2.loadSettings(settings)
        self.assertEqual(handler2.baudrateEdit.text(), "230400")
        self.assertTrue(handler2.unitCheckBox.isChecked())

        recorder2 = DataRecorder(device, "COM3")
        recorder2.loadSettings(settings)
        self.assertEqual(recorder2.saveDurationEdit.text(), "120")
        self.assertEqual(recorder2.filePrefixEdit.text(), "TrialA")
        self.assertTrue(recorder2.continuousCheckBox.isChecked())
        self.assertFalse(recorder2.virtualObjectCheckBox.isChecked())


if __name__ == "__main__":
    unittest.main()

