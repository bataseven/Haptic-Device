import unittest

from PyQt5.QtCore import QEventLoop, QTimer

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


class SerialWorkerThreadTest(unittest.TestCase):
    """The worker in its own thread, as in the app."""

    def setUp(self):
        SerialHandler.useWorkerThread = True
        self.addCleanup(setattr, SerialHandler, "useWorkerThread", False)
        self.handler = SerialHandler(HapticDevice())
        self.addCleanup(self.handler.shutdown)

    def test_runs_in_its_own_thread(self):
        self.assertTrue(self.handler.workerThread.isRunning())
        self.assertIs(self.handler.serialWorker.thread(), self.handler.workerThread)
        self.assertIs(self.handler.serialWorker.serial.thread(), self.handler.workerThread)

    def test_failed_open_is_reported_and_the_button_is_enabled_again(self):
        statuses = []
        self.handler.serialWorker.connectionStatusChanged.connect(statuses.append)
        self.handler.portsCombo.addItem("COM_DOES_NOT_EXIST", "COM_DOES_NOT_EXIST")
        self.handler.portsCombo.setCurrentIndex(self.handler.portsCombo.count() - 1)
        self.handler.connectOrDisconnect()

        loop = QEventLoop()
        self.handler.serialWorker.connectionStatusChanged.connect(lambda status: loop.quit())
        QTimer.singleShot(5000, loop.quit)
        loop.exec_()
        self.assertEqual(statuses, [ConnectionStatus.DeviceNotFound])
        self.assertEqual(self.handler.connectSerialButton.text(), "Connect")
        self.assertTrue(self.handler.connectSerialButton.isEnabled())

    def test_shutdown_stops_the_thread(self):
        self.handler.shutdown()
        self.assertFalse(self.handler.workerThread.isRunning())


if __name__ == "__main__":
    unittest.main()
