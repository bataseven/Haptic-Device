from PyQt5 import QtWidgets

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.widget = QtWidgets.QWidget()
        self.setCentralWidget(self.widget)

class MainMenu(QtWidgets.QMenuBar):
    def __init__(self):
        super().__init__()

        self.new_window_action = QtWidgets.QAction("New Window", self)
        self.new_window_action.triggered.connect(self.open_new_window)
        self.addAction(self.new_window_action)

    def open_new_window(self):
        new_window = MainWindow()
        new_window.show()

if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    main_window = MainWindow()
    main_menu = MainMenu()
    main_window.setMenuBar(main_menu)
    main_window.show()
    app.exec_()
