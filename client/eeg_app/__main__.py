#!/usr/bin/env python3
"""Entry point for the unified EEG acquisition app."""
import sys
import ctypes

# Windows high-resolution timer
if sys.platform == "win32":
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

from PyQt5.QtWidgets import QApplication
from eeg_app.ui.theme import APP_STYLESHEET
from eeg_app.app import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(APP_STYLESHEET)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
