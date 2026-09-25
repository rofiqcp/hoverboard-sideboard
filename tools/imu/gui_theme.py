from PySide6.QtGui import QColor

BG = "#0b1118"
PANEL = "#121a24"
PANEL_2 = "#17212d"
BORDER = "#243244"
TEXT = "#e8f0f8"
MUTED = "#8ea0b5"
ACCENT = "#45b6ff"
ACCENT_2 = "#7ce7c4"
WARN = "#ffc857"
DANGER = "#ff6b6b"
GOOD = "#69db7c"

STYLE = r"""
QMainWindow, QWidget {
    background: #0b1118;
    color: #e8f0f8;
    font-family: "Segoe UI", "Inter", sans-serif;
    font-size: 10pt;
}
QFrame#Card {
    background: #121a24;
    border: 1px solid #243244;
    border-radius: 14px;
}
QFrame#TopBar {
    background: #0f1721;
    border-bottom: 1px solid #243244;
}
QFrame#SideBar {
    background: #0d141d;
    border-right: 1px solid #243244;
}
QLabel#Title {
    font-size: 18pt;
    font-weight: 700;
}
QLabel#SectionTitle {
    font-size: 12pt;
    font-weight: 700;
}
QLabel#MetricValue {
    font-size: 20pt;
    font-weight: 700;
}
QLabel#Muted {
    color: #8ea0b5;
}
QPushButton {
    background: #17212d;
    border: 1px solid #2b3a4d;
    border-radius: 10px;
    padding: 8px 12px;
    color: #e8f0f8;
}
QPushButton:hover {
    background: #1d2a38;
    border-color: #3a5068;
}
QPushButton:pressed {
    background: #111923;
}
QPushButton#Primary {
    background: #168ad0;
    border-color: #45b6ff;
    font-weight: 700;
}
QPushButton#Primary:hover { background: #1f9de8; }
QPushButton#Danger {
    background: #402029;
    border-color: #783441;
    color: #ffd9df;
}
QPushButton#Nav {
    text-align: left;
    padding: 10px 14px;
    border: 0;
    background: transparent;
    color: #9fb0c4;
}
QPushButton#Nav:checked {
    background: #172536;
    color: #ffffff;
    border-left: 3px solid #45b6ff;
    border-radius: 7px;
}
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {
    background: #101923;
    border: 1px solid #2b3a4d;
    border-radius: 8px;
    padding: 6px 9px;
    min-height: 22px;
}
QProgressBar {
    background: #0f1721;
    border: 1px solid #243244;
    border-radius: 7px;
    text-align: center;
    min-height: 14px;
}
QProgressBar::chunk {
    background: #45b6ff;
    border-radius: 6px;
}
QTableWidget {
    background: #101821;
    alternate-background-color: #131d28;
    gridline-color: #243244;
    border: 1px solid #243244;
    border-radius: 10px;
}
QHeaderView::section {
    background: #17212d;
    color: #aebed0;
    border: 0;
    padding: 7px;
}
QTabWidget::pane { border: 0; }
QTabBar::tab {
    background: #111923;
    color: #8ea0b5;
    padding: 8px 12px;
    border-radius: 8px;
    margin-right: 4px;
}
QTabBar::tab:selected {
    background: #1b2b3d;
    color: #ffffff;
}
QToolTip {
    background: #1b2735;
    color: #ffffff;
    border: 1px solid #3a5068;
    padding: 5px;
}
"""
