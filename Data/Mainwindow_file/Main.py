
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow
from Serial_Main_Backend import Serial_Main_Backend



if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = Serial_Main_Backend(apps=app)
    sys.exit(app.exec_())

