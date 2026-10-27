import importlib.machinery
import importlib.util
import os
import sys
import unittest
from unittest import mock

import helpers
from PyQt5.QtWidgets import QMessageBox


def loadApp():
    """Loads app.pyw as a module. Its main() is not run."""
    loader = importlib.machinery.SourceFileLoader("app", os.path.join(helpers.APP_DIR, "app.pyw"))
    module = importlib.util.module_from_spec(importlib.util.spec_from_loader("app", loader))
    loader.exec_module(module)
    return module


class ErrorHandlerTest(unittest.TestCase):
    def setUp(self):
        self.addCleanup(setattr, sys, "excepthook", sys.excepthook)  # Loading app.pyw replaces it
        self.app = loadApp()

    def test_shows_each_error_once(self):
        with mock.patch.object(QMessageBox, "exec_", return_value=0) as dialog, \
                mock.patch("sys.__excepthook__"):
            for _ in range(3):
                self.app.showUnhandledException(ZeroDivisionError, ZeroDivisionError("division by zero"), None)
            self.app.showUnhandledException(KeyError, KeyError("x"), None)
        self.assertEqual(dialog.call_count, 2)

    def test_is_installed(self):
        self.assertIs(sys.excepthook, self.app.showUnhandledException)


if __name__ == "__main__":
    unittest.main()
