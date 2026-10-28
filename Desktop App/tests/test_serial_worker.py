import unittest

from PyQt5.QtCore import QCoreApplication, QEventLoop, QTimer
from PyQt5.QtSerialPort import QSerialPortInfo

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

    def test_button_is_disabled_while_connecting_and_enabled_after_a_failed_open(self):
        statuses = []
        self.handler.serialWorker.connectionStatusChanged.connect(statuses.append)
        self.handler.portsCombo.addItem("COM_DOES_NOT_EXIST", "COM_DOES_NOT_EXIST")
        self.handler.portsCombo.setCurrentIndex(self.handler.portsCombo.count() - 1)
        self.handler.connectOrDisconnect()
        # Refreshing the ports would enable the button while connecting
        self.assertFalse(self.handler.refreshPortsTimer.isActive())
        self.assertFalse(self.handler.connectSerialButton.isEnabled())
        self.assertFalse(self.handler.portsCombo.isEnabled())
        self.assertFalse(self.handler.refreshPortsButton.isEnabled())
        self.handler.refreshSerialPorts()  # e.g. the refresh button was clicked before it was disabled
        self.assertFalse(self.handler.connectSerialButton.isEnabled())
        self.assertFalse(self.handler.portsCombo.isEnabled())

        loop = QEventLoop()
        self.handler.serialWorker.connectionStatusChanged.connect(lambda status: loop.quit())
        QTimer.singleShot(5000, loop.quit)
        loop.exec_()
        QCoreApplication.processEvents()  # Let the handler handle the status, which is queued to its thread
        self.assertEqual(statuses, [ConnectionStatus.DeviceNotFound])
        self.assertEqual(self.handler.connectSerialButton.text(), "Connect")
        # The button is enabled again if there is a port to connect to
        hasPorts = len(QSerialPortInfo.availablePorts()) > 0
        self.assertEqual(self.handler.connectSerialButton.isEnabled(), hasPorts)
        self.assertTrue(self.handler.refreshPortsTimer.isActive())

    def test_shutdown_stops_the_thread(self):
        self.handler.shutdown()
        self.assertFalse(self.handler.workerThread.isRunning())


if __name__ == "__main__":
    unittest.main()
