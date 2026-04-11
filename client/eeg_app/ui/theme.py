"""Application theme — stylesheets, colors, fonts."""
from eeg_app.core.constants import (
    DARK_BG, SURFACE, SURFACE_RAISED, BORDER, BORDER_FOCUS,
    TEXT_PRIMARY, TEXT_SECONDARY, TEXT_MUTED, ACCENT, ACCENT_DIM,
    FONT_FAMILY,
)

APP_STYLESHEET = f"""
QMainWindow, QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_PRIMARY};
    font-family: {FONT_FAMILY};
    font-size: 14px;
}}
QLabel {{
    color: {TEXT_PRIMARY};
    background: transparent;
}}
QLineEdit {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 6px 10px;
    font-size: 14px;
}}
QLineEdit:focus {{
    border-color: {BORDER_FOCUS};
}}
QSpinBox, QDoubleSpinBox {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 14px;
}}
QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {BORDER_FOCUS};
}}
QComboBox {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 14px;
}}
QComboBox QAbstractItemView {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    selection-background-color: {ACCENT_DIM};
}}
QCheckBox {{
    color: {TEXT_PRIMARY};
    spacing: 8px;
    font-size: 14px;
}}
QCheckBox::indicator {{
    width: 18px;
    height: 18px;
    border: 1px solid {BORDER};
    border-radius: 3px;
    background: {SURFACE_RAISED};
}}
QCheckBox::indicator:checked {{
    background: {ACCENT_DIM};
    border-color: {ACCENT};
}}
QRadioButton {{
    color: {TEXT_PRIMARY};
    spacing: 8px;
    font-size: 14px;
}}
QRadioButton::indicator {{
    width: 18px;
    height: 18px;
    border: 1px solid {BORDER};
    border-radius: 9px;
    background: {SURFACE_RAISED};
}}
QRadioButton::indicator:checked {{
    background: {ACCENT_DIM};
    border-color: {ACCENT};
}}
QPushButton {{
    background-color: {SURFACE_RAISED};
    color: {TEXT_PRIMARY};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 8px 20px;
    font-size: 14px;
    font-weight: bold;
}}
QPushButton:hover {{
    background-color: #3a3a3a;
    border-color: #555555;
}}
QPushButton:pressed {{
    background-color: #1a1a1a;
}}
QPushButton:disabled {{
    color: {TEXT_MUTED};
    background-color: #1a1a1a;
    border-color: #222222;
}}
QProgressBar {{
    background-color: {SURFACE};
    border: none;
    border-radius: 3px;
    max-height: 6px;
}}
QProgressBar::chunk {{
    background-color: {ACCENT_DIM};
    border-radius: 3px;
}}
QScrollArea {{
    border: none;
    background: transparent;
}}
QFrame {{
    border: none;
}}
"""
