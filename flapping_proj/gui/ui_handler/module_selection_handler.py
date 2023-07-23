from PyQt6 import QtWidgets

from gui.ui_handler.grass_handler import GrassHandler

Form, Window = uic.loadUiType("main.ui")

app = QApplication([])
window = Window()
form = Form()
form.setupUi(window)
window.show()
app.exec()