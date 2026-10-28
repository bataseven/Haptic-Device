import unittest

import helpers  # noqa: F401  Creates the Qt application
from UI.devices import HapticDevice
from UI.serial_handler import SerialHandler, findMessages
from UI.settings import serialSettings as st


class SerialHandlerParserTest(unittest.TestCase):
    """Parsing of the messages received from the device."""

    def setUp(self):
        self.device = HapticDevice()
        self.handler = SerialHandler(self.device)
        self.sent = []
        self.handler.messageToSend.connect(self.sent.append)

    def test_status_values_are_hexadecimal(self):
        self.handler.parseMessage("<H01A#-3><H1-B#2>")
        self.assertEqual((self.device.fingerPos, self.device.fingerVel), (26, -3))
        self.assertEqual((self.device.palmPos, self.device.palmVel), (-11, 2))

    def test_new_data_is_emitted_after_the_line_is_parsed(self):
        seen = []
        self.handler.newData.connect(lambda: seen.append(self.device.fingerPos))
        self.handler.parseMessage("<H014#0><H10#0>")
        self.assertEqual(seen, [20])

    def test_lines_without_status_are_not_new_data(self):
        seen = []
        self.handler.newData.connect(lambda: seen.append(True))
        self.handler.parseMessage("<K01#2#3#4#5#6#7#8#9>")
        self.assertEqual(seen, [])
        self.assertEqual(self.device.fingerGains, [1, 2, 3, 4, 5, 6, 7, 8, 9])

    def test_forces(self):
        self.handler.parseMessage("<N1.5><F0.1#0.2#0.3#0.4>")
        self.assertEqual(self.device.netForce, 1.5)
        self.assertEqual(self.device.forceValues, [0.1, 0.2, 0.3, 0.4])

    def test_controller_modes(self):
        self.handler.parseMessage("<M01><M12>")
        self.assertEqual((self.device.fingerControlMode, self.device.palmControlMode), (1, 2))
        self.assertEqual(self.handler.fingerModeLabel.text(), "Velocity")

    def test_unknown_controller_mode_is_ignored(self):
        self.handler.parseMessage("<M05>")
        self.assertEqual(self.device.fingerControlMode, 2)

    def test_unit_factors(self):
        oldFactors = (st.COUNT_TO_MM, st.COUNT_TO_MM2)
        self.addCleanup(setattr, st, "COUNT_TO_MM", oldFactors[0])
        self.addCleanup(setattr, st, "COUNT_TO_MM2", oldFactors[1])
        self.handler.parseMessage("<U0.001#0.01>")
        self.assertEqual((st.COUNT_TO_MM, st.COUNT_TO_MM2), (0.001, 0.01))

    def test_device_time(self):
        self.handler.parseMessage("<R2500000>")
        self.assertEqual(self.device.deviceTime, 2.5)

    def test_host_timeout_resets_the_waveforms_and_rearms_the_timeout(self):
        self.device.waveformParams[1].update({'T': 'SQUARE', 'F': 2, 'A': 5000})
        self.handler.parseMessage("<X1000>")
        self.assertEqual(self.device.waveformParams[1]['T'], 'CONSTANT')
        self.assertEqual(self.device.waveformParams[1]['A'], 0)
        self.assertIn(f"T{st.HOST_TIMEOUT_MS}", self.sent)

    def test_malformed_messages_do_not_raise(self):
        self.handler.parseMessage("<Hxyz><F1#a><M><Rabc><U1>")
        self.assertIn("Error Parsing", self.handler.infoMessagesEdit.toPlainText())

    def test_find_messages(self):
        self.assertEqual(findMessages("x<A1>y<B2#3>"), [("<A1>", "A1"), ("<B2#3>", "B2#3")])


class SerialHandlerWaveParameterTest(unittest.TestCase):
    """Sending the waveform parameters typed in the main window."""

    def setUp(self):
        self.device = HapticDevice()
        self.handler = SerialHandler(self.device)
        self.sent = []
        self.handler.messageToSend.connect(self.sent.append)
        self.handler.waveRadio1.setChecked(True)
        self.handler.unitCheckBox.setChecked(False)
        oldFactor = st.COUNT_TO_MM
        self.addCleanup(setattr, st, "COUNT_TO_MM", oldFactor)

    def send(self, value, paramType):
        self.handler.ampEdit.setText(value)
        self.handler.sendWaveParameter(self.handler.ampEdit, paramType)

    def test_whole_numbers_are_sent_as_int(self):
        self.send("5000", 'A')
        self.assertEqual(self.device.waveformParams[0]['A'], 5000)
        self.assertEqual(self.sent, ["WA0#5000"])

    def test_decimals_are_kept(self):
        self.send("0.5", 'F')
        self.assertEqual(self.sent, ["WF0#0.5"])

    def test_millimeters_are_converted_to_counts(self):
        st.COUNT_TO_MM = 0.01
        self.handler.unitCheckBox.setChecked(True)
        self.send("2", 'A')
        self.assertEqual(self.sent, ["WA0#200"])

    def test_zero_conversion_factor_is_not_sent(self):
        st.COUNT_TO_MM = 0
        self.handler.unitCheckBox.setChecked(True)
        self.send("2", 'A')
        self.assertEqual(self.sent, [])
        self.assertIn("conversion factor is 0", self.handler.infoMessagesEdit.toPlainText())


if __name__ == "__main__":
    unittest.main()
