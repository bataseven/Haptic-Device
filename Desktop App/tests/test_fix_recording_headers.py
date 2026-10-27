import unittest

import helpers  # noqa: F401  Sets up the import paths
from fix_recording_headers import fixText

# Header of a recording made before the gain columns were added
OLD_HEADER = ("Data Point, Time, Finger Position, Finger Velocity, Finger Control Mode, Finger Wave Type, "
              "Finger Wave Amplitude, Finger Wave Frequency, Finger Wave Offset, Finger Wave Value, "
              "Palm Position, Palm Velocity, Palm Control Mode, Palm Wave Type, Palm Wave Amplitude, "
              "Palm Wave Frequency, Palm Wave Offset, Palm Wave Value, "
              "Thumb Outer Force, Thumb Inner Force, Index Inner Force, Index Outer Force")
# The palm wave is a 0.3 Hz square wave with an amplitude of 5000
ROW = "0, 1673021522.823, 0, 0, 2, SINUSOIDAL,0,0,0,0.0, 1652, 10131, 0, SQUARE,0.3,5000,0,5000.0, 0.0,0.0,0.0,0.0"


def columns(line):
    return [value.strip() for value in line.split(",")]


class FixRecordingHeadersTest(unittest.TestCase):
    def test_swaps_the_amplitude_and_frequency_labels(self):
        header, row = fixText(OLD_HEADER + "\n" + ROW + "\n").splitlines()
        values = dict(zip(columns(header), columns(row)))
        self.assertEqual(values["Palm Wave Frequency"], "0.3")
        self.assertEqual(values["Palm Wave Amplitude"], "5000")

    def test_moves_the_first_row_out_of_the_header_line(self):
        lines = fixText(OLD_HEADER + ROW + "\n" + ROW + "\n").splitlines()
        self.assertEqual(len(lines), 3)
        self.assertEqual(lines[1], ROW)

    def test_adds_missing_gain_columns(self):
        gains = "0,0,0,0,0,0,0,0,0, "
        row = ("0, 1, 0, 0, 2, " + gains + "SINUSOIDAL,0,0,0,0.0, 0, 0, 0, " + gains +
               "SQUARE,0.3,5000,0,5000.0, 0.0,0.0,0.0,0.0")
        header = fixText(OLD_HEADER + "\n" + row + "\n").splitlines()[0]
        self.assertEqual(len(columns(header)), len(columns(row)))
        self.assertIn("Palm PID Force D", header)

    def test_leaves_fixed_files_unchanged(self):
        fixed = fixText(OLD_HEADER + "\n" + ROW + "\n")
        self.assertEqual(fixText(fixed), fixed)

    def test_leaves_new_recordings_unchanged(self):
        text = "Data Point, Time, Index Outer Force, Device Time\n0, 1, 0.0, 0.5\n"
        self.assertEqual(fixText(text), text)


if __name__ == "__main__":
    unittest.main()
