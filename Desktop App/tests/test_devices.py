import unittest

import helpers  # noqa: F401  Sets up the import paths
from UI.devices import HapticDevice


class DeviceTimeTest(unittest.TestCase):
    def test_converts_microseconds_to_seconds(self):
        device = HapticDevice()
        device.setDeviceTime(1_500_000)
        self.assertEqual(device.deviceTime, 1.5)

    def test_keeps_counting_when_micros_overflows(self):
        device = HapticDevice()
        device.setDeviceTime(2 ** 32 - 1000)
        device.setDeviceTime(500)
        self.assertAlmostEqual(device.deviceTime, (2 ** 32 + 500) / 1e6)

    def test_starts_from_zero_when_the_device_restarts(self):
        device = HapticDevice()
        device.setDeviceTime(10_000_000)
        device.setDeviceTime(2_000)
        self.assertEqual(device.deviceTime, 0.002)


if __name__ == "__main__":
    unittest.main()
