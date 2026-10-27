import unittest

from helpers import FakeSerialPort
from UI.devices import HapticDevice
from UI.serial_handler import SerialHandler
from UI.serial_worker import ConnectionStatus


class SerialWorkerTest(unittest.TestCase):
    def setUp(self):
        self.device = HapticDevice()
        self.handler = SerialHandler(self.device)
        self.worker = self.handler.serialWorker
        self.port = FakeSerialPort()
        self.worker.serial = self.port

    def test_heartbeat_is_sent_but_not_logged(self):
        logged = []
        self.worker.sentMessage.connect(logged.append)
        self.handler.handleConnectionChanges(ConnectionStatus.Connected)
        self.assertTrue(self.handler.heartbeatTimer.isActive())

        self.handler.heartbeatTimer.timeout.emit()
        self.assertEqual(self.port.written, ["<A>"])
        self.assertEqual(logged, [])

    def test_disconnecting_disarms_the_host_timeout(self):
        self.handler.handleConnectionChanges(ConnectionStatus.Connected)
        self.worker.disconnectSerial()
        self.assertEqual(self.port.written, ["<T0>", "<C0>"])
        self.assertFalse(self.port.open)
        self.assertFalse(self.handler.heartbeatTimer.isActive())

    def test_corrupted_bytes_do_not_raise(self):
        self.port.lines = [b"\xff\xfe<H01A#0>\r\n"]
        self.worker.readSerial()
        self.assertEqual(self.device.fingerPos, 26)


if __name__ == "__main__":
    unittest.main()
