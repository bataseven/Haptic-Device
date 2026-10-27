import glob
import os
import shutil
import tempfile
import unittest
from unittest import mock

import helpers  # noqa: F401  Creates the Qt application
from PyQt5.QtWidgets import QMessageBox
from UI.data_recorder import DataRecorder
from UI.devices import HapticDevice
from UI.serial_handler import SerialHandler

STATUS_LINE = "<R{time}><H0{finger:X}#1><H1{palm:X}#2><N1.5><F0.1#0.2#0.3#0.4><W00.0><W15000.0>"


class DataRecorderTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.folder)
        dialog = mock.patch.object(QMessageBox, "exec_", return_value=0)  # The "Recording done" dialog
        dialog.start()
        self.addCleanup(dialog.stop)

        self.device = HapticDevice()
        self.handler = SerialHandler(self.device)
        self.recorder = DataRecorder(self.device, "TEST")
        self.recorder.savePathEdit.setText(self.folder)
        self.handler.newData.connect(self.recorder.appendToFile)

    def record(self, *lines):
        self.recorder.startRecording()
        for line in lines:
            self.handler.parseMessage(line)
        self.recorder.stopRecording()

    def readRows(self):
        path = glob.glob(os.path.join(self.folder, "*.csv"))[0]
        with open(path) as file:
            lines = file.read().splitlines()
        header = [name.strip() for name in lines[0].split(",")]
        rows = [[value.strip() for value in line.split(",")] for line in lines[1:]]
        return header, rows

    def test_each_row_has_the_values_of_its_line(self):
        self.record(STATUS_LINE.format(time=1000000, finger=10, palm=11),
                    STATUS_LINE.format(time=1025000, finger=20, palm=21))
        header, rows = self.readRows()
        rows = [dict(zip(header, row)) for row in rows]
        self.assertEqual([row["Finger Position"] for row in rows], ["10", "20"])
        self.assertEqual([row["Device Time"] for row in rows], ["1.000000", "1.025000"])

    def test_header_and_rows_have_the_same_columns(self):
        self.record(STATUS_LINE.format(time=0, finger=1, palm=2))
        header, rows = self.readRows()
        self.assertEqual(len(header), len(rows[0]))

    def test_wave_columns_match_their_values(self):
        self.device.waveformParams[1].update({'T': 'SQUARE', 'F': 0.3, 'A': 5000, 'O': 10})
        self.record(STATUS_LINE.format(time=0, finger=1, palm=2))
        header, rows = self.readRows()
        row = dict(zip(header, rows[0]))
        self.assertEqual(row["Palm Wave Frequency"], "0.3")
        self.assertEqual(row["Palm Wave Amplitude"], "5000")
        self.assertEqual(row["Palm Wave Offset"], "10")

    def test_lines_without_status_are_not_recorded(self):
        self.record(STATUS_LINE.format(time=0, finger=1, palm=2), "<K01#2#3#4#5#6#7#8#9>")
        _, rows = self.readRows()
        self.assertEqual(len(rows), 1)

    def test_empty_duration_does_not_start_recording(self):
        self.recorder.saveDurationEdit.setText("")
        with mock.patch.object(QMessageBox, "warning") as warning:
            self.recorder.startRecording()
        warning.assert_called_once()
        self.assertFalse(self.recorder.isRecording)
        self.assertEqual(glob.glob(os.path.join(self.folder, "*.csv")), [])


if __name__ == "__main__":
    unittest.main()
