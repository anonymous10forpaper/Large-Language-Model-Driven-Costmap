

APP_STYLESHEET = """
QWidget {
    color: #202936;
    font-family: "Noto Sans", "DejaVu Sans", sans-serif;
    font-size: 13px;
}

QMainWindow, QWidget#appRoot, QStackedWidget#pageStack {
    background: #eef1f5;
}

QFrame#sidebar {
    background: #151d29;
    border: 0;
}

QLabel#brandMark {
    background: #3674d9;
    color: #ffffff;
    border-radius: 6px;
    font-size: 15px;
    font-weight: 700;
}

QLabel#brandName {
    color: #f8fafc;
    font-size: 15px;
    font-weight: 700;
}

QLabel#brandSubtitle, QLabel#sidebarMeta {
    color: #93a1b4;
    font-size: 10px;
}

QPushButton#navButton {
    background: transparent;
    color: #b7c2d1;
    border: 0;
    border-radius: 6px;
    min-height: 40px;
    padding: 0 12px;
    text-align: left;
    font-weight: 500;
}

QPushButton#navButton:hover {
    background: #202c3c;
    color: #ffffff;
}

QPushButton#navButton:checked {
    background: #346fc8;
    color: #ffffff;
    font-weight: 600;
}

QFrame#headerBar {
    background: #f7f9fb;
    border-bottom: 1px solid #d4dae3;
}

QLabel#pageTitle {
    color: #1d2734;
    font-size: 19px;
    font-weight: 700;
}

QLabel#pageSubtitle {
    color: #697586;
    font-size: 11px;
}

QFrame#surface, QFrame#metricTile, QFrame#modelMetricTile {
    background: #ffffff;
    border: 1px solid #d8dee8;
    border-radius: 7px;
}

QFrame#modelMetricTile:hover, QFrame#modelMetricTile:focus {
    background: #f8fbff;
    border-color: #86a7d6;
}

QFrame#compactModelSelector {
    background: #ffffff;
    border: 1px solid #cbd3df;
    border-radius: 5px;
}

QFrame#compactModelSelector:hover, QFrame#compactModelSelector:focus {
    background: #f8fbff;
    border-color: #86a7d6;
}

QLabel#sectionTitle {
    color: #273240;
    font-size: 14px;
    font-weight: 650;
}

QLabel#sectionMeta, QLabel#fieldLabel {
    color: #778395;
    font-size: 10px;
    font-weight: 600;
}

QLabel#metricLabel {
    color: #778395;
    font-size: 10px;
    font-weight: 600;
}

QLabel#metricValue {
    color: #202936;
    font-size: 14px;
    font-weight: 700;
}

QLabel#mapPath {
    color: #566476;
    font-family: "DejaVu Sans Mono", monospace;
    font-size: 10px;
}

QLabel#statusOnline {
    background: #e6f5ed;
    color: #237c50;
    border: 1px solid #a9dcc1;
    border-radius: 6px;
    padding: 5px 9px;
    font-size: 11px;
    font-weight: 600;
}

QLabel#statusOffline {
    background: #fcecec;
    color: #a53636;
    border: 1px solid #efb9b9;
    border-radius: 6px;
    padding: 5px 9px;
    font-size: 11px;
    font-weight: 600;
}

QLabel#statusBusy {
    background: #fff4dc;
    color: #94620f;
    border: 1px solid #ebce8b;
    border-radius: 6px;
    padding: 5px 9px;
    font-size: 11px;
    font-weight: 600;
}

QPlainTextEdit#commandInput {
    background: #fbfcfe;
    color: #1d2734;
    border: 1px solid #cbd3df;
    border-radius: 6px;
    padding: 10px;
    selection-background-color: #3674d9;
    font-size: 14px;
}

QPlainTextEdit#commandInput:focus {
    border: 2px solid #3674d9;
    padding: 9px;
}

QPlainTextEdit#compactCommandInput {
    background: #fbfcfe;
    color: #1d2734;
    border: 1px solid #cbd3df;
    border-radius: 6px;
    padding: 8px;
    selection-background-color: #3674d9;
    font-size: 13px;
}

QPlainTextEdit#compactCommandInput:focus {
    border: 2px solid #3674d9;
    padding: 7px;
}

QLabel#robotName {
    color: #1f2c3b;
    font-size: 16px;
    font-weight: 700;
}

QLabel#robotPolicyLine {
    color: #435064;
    font-size: 11px;
    padding: 1px 0;
}

QTextEdit#compactActivityLog {
    background: #f7f9fc;
    color: #3a4655;
    border: 1px solid #e0e5ec;
    border-radius: 5px;
    padding: 5px 7px;
    font-family: "DejaVu Sans Mono", monospace;
    font-size: 10px;
}

QComboBox, QSpinBox {
    background: #ffffff;
    color: #273240;
    border: 1px solid #cbd3df;
    border-radius: 5px;
    min-height: 30px;
    padding: 0 8px;
}

QComboBox:hover, QSpinBox:hover {
    border-color: #8fa1b8;
}

QComboBox::drop-down {
    border: 0;
    width: 24px;
}

QComboBox QAbstractItemView {
    background: #ffffff;
    border: 1px solid #cbd3df;
    selection-background-color: #e7eef9;
    selection-color: #1d2734;
}

QPushButton#primaryButton {
    background: #3674d9;
    color: #ffffff;
    border: 1px solid #2d64bd;
    border-radius: 6px;
    min-height: 34px;
    padding: 0 16px;
    font-weight: 650;
}

QPushButton#primaryButton:hover {
    background: #2e68c6;
}

QPushButton#primaryButton:pressed {
    background: #285bad;
}

QPushButton#primaryButton:disabled {
    background: #aeb9c8;
    border-color: #aeb9c8;
}

QPushButton#secondaryButton {
    background: #ffffff;
    color: #344052;
    border: 1px solid #cbd3df;
    border-radius: 6px;
    min-height: 32px;
    padding: 0 12px;
    font-weight: 600;
}

QPushButton#secondaryButton:hover,
QPushButton#secondaryButton:checked {
    background: #e9f0fb;
    color: #2f63ad;
    border-color: #9eb8dc;
}

QPushButton#secondaryButton:disabled {
    color: #a6afb9;
    background: #f5f6f8;
    border-color: #d9dee5;
}

QPushButton#dangerButton {
    background: #ffffff;
    color: #b43d3d;
    border: 1px solid #e1adad;
    border-radius: 6px;
    min-height: 32px;
    padding: 0 12px;
    font-weight: 600;
}

QPushButton#dangerButton:hover {
    background: #fff0f0;
}

QPushButton#dangerButton:disabled {
    color: #a6afb9;
    border-color: #d9dee5;
}

QToolButton {
    background: #ffffff;
    color: #4a586a;
    border: 1px solid #cbd3df;
    border-radius: 5px;
    min-width: 32px;
    min-height: 32px;
}

QToolButton:hover {
    background: #f0f4f9;
    border-color: #8fa1b8;
}

QToolButton#modelArrow {
    background: transparent;
    border: 0;
    min-width: 26px;
    min-height: 26px;
}

QToolButton#modelArrow:hover {
    background: #e8eef7;
    border: 0;
}

QMenu {
    background: #ffffff;
    color: #273240;
    border: 1px solid #cbd3df;
    padding: 5px;
}

QMenu::item {
    min-height: 26px;
    padding: 4px 28px 4px 10px;
    border-radius: 4px;
}

QMenu::item:selected {
    background: #e7eef9;
    color: #1d2734;
}

QMenu::item:disabled {
    color: #8b96a5;
}

QMenu::separator {
    background: #e1e6ed;
    height: 1px;
    margin: 4px 6px;
}

QGraphicsView#waypointMapView {
    background: #343a43;
    border: 1px solid #cbd3df;
    border-radius: 4px;
}

QListWidget#waypointList {
    background: #fbfcfe;
    color: #2a3442;
    border: 1px solid #d8dee8;
    border-radius: 5px;
    outline: 0;
    padding: 3px;
}

QListWidget#waypointList::item {
    min-height: 29px;
    padding: 2px 8px;
    border-radius: 4px;
}

QListWidget#waypointList::item:selected {
    background: #dfeafb;
    color: #245d9f;
    font-weight: 650;
}

QLabel#mapStatusNeutral, QLabel#mapStatusOnline,
QLabel#mapStatusBusy, QLabel#mapStatusError {
    border-radius: 5px;
    padding: 7px 8px;
    font-size: 10px;
    font-weight: 600;
}

QLabel#mapStatusNeutral {
    background: #f1f3f6;
    color: #677486;
}

QLabel#mapStatusOnline {
    background: #e6f5ed;
    color: #237c50;
}

QLabel#mapStatusBusy {
    background: #fff4dc;
    color: #94620f;
}

QLabel#mapStatusError {
    background: #fcecec;
    color: #a53636;
}

QLabel#zoneInactive {
    background: #f2f4f7;
    color: #8a95a4;
    border: 1px solid #d6dce5;
    border-radius: 5px;
    font-size: 13px;
    font-weight: 700;
}

QLabel#zoneForbidden {
    background: #fbe8e8;
    color: #ad3434;
    border: 1px solid #e7a8a8;
    border-radius: 5px;
    font-size: 13px;
    font-weight: 700;
}

QLabel#routeTag {
    background: #e9f0fb;
    color: #2f63ad;
    border: 1px solid #bfd0ea;
    border-radius: 5px;
    padding: 5px 8px;
    font-weight: 650;
}

QLabel#objectRule {
    background: #e8f5ee;
    color: #28754f;
    border: 1px solid #b7dcc9;
    border-radius: 5px;
    padding: 5px 8px;
    font-weight: 600;
}

QLabel#emptyValue {
    color: #9aa4b1;
    font-style: italic;
}

QProgressBar {
    background: #edf0f4;
    border: 0;
    border-radius: 4px;
    min-height: 10px;
    max-height: 10px;
    text-align: center;
}

QProgressBar::chunk {
    background: #3674d9;
    border-radius: 4px;
}

QProgressBar#missionProgress {
    min-height: 7px;
    max-height: 7px;
}

QLabel#stageDone {
    color: #2f8157;
    font-weight: 600;
}

QLabel#stageActive {
    color: #2f69bd;
    font-weight: 700;
}

QLabel#stagePending {
    color: #9aa4b1;
}

QLabel#stageFailed {
    color: #b43d3d;
    font-weight: 700;
}

QTextEdit#activityLog, QPlainTextEdit#jsonView {
    background: #f8fafc;
    color: #2a3442;
    border: 1px solid #dce2ea;
    border-radius: 5px;
    padding: 7px;
    font-family: "DejaVu Sans Mono", monospace;
    font-size: 10px;
    selection-background-color: #c7d8f2;
}

QTableWidget {
    background: #ffffff;
    alternate-background-color: #f7f9fc;
    color: #283342;
    border: 1px solid #d8dee8;
    border-radius: 5px;
    gridline-color: #e3e7ed;
    selection-background-color: #dbe7f8;
    selection-color: #1e2936;
}

QHeaderView::section {
    background: #edf1f6;
    color: #5d6979;
    border: 0;
    border-bottom: 1px solid #d4dae3;
    border-right: 1px solid #dce2e9;
    padding: 7px;
    font-size: 10px;
    font-weight: 650;
}

QTabWidget::pane {
    border: 1px solid #d8dee8;
    background: #ffffff;
    border-radius: 5px;
}

QTabBar::tab {
    background: #e9edf2;
    color: #697586;
    border: 1px solid #d8dee8;
    border-bottom: 0;
    padding: 7px 13px;
    margin-right: 2px;
}

QTabBar::tab:selected {
    background: #ffffff;
    color: #2f69bd;
    font-weight: 650;
}

QCheckBox {
    color: #344052;
    spacing: 7px;
}

QCheckBox::indicator {
    width: 16px;
    height: 16px;
}

QScrollBar:vertical {
    background: transparent;
    width: 10px;
    margin: 2px;
}

QScrollBar::handle:vertical {
    background: #bcc5d0;
    min-height: 28px;
    border-radius: 4px;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}

QSplitter::handle {
    background: #eef1f5;
}
"""
