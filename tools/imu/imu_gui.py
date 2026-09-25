"""Interactive Qt6 dashboard for the hoverboard sideboard IMU."""
import csv
import math
import os
import queue
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

from PySide6.QtCore import Qt, QThread, Signal, QTimer, QSettings, QUrl
from PySide6.QtGui import QDesktopServices, QFont, QIcon, QKeySequence, QShortcut, QColor
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPushButton, QProgressBar, QScrollArea,
    QSizePolicy, QStackedWidget, QTableWidget, QTableWidgetItem, QVBoxLayout,
    QWidget
)

from read_imu import read_frame, decode_imu, flag_text, nav_text
from calibrate_imu import packet as cal_packet, parse_cal_status, COMM_CAL, CMDS
from configure_imu import packet as cfg_packet, decode as decode_config, COMM as CONFIG_COMM, SUB as CONFIG_SUB
from serial_common import open_sideboard_port
from tool_paths import log_output, display_path, LOG_DIR
from gui_theme import STYLE, ACCENT, ACCENT_2, DANGER, GOOD, MUTED, WARN
from gui_widgets import (
    Attitude3DWidget, ArtificialHorizonWidget, LivePlotWidget,
    MetricCard, StatusPill, VectorBar,
)

G = 9.80665
CAL_STATE = {0: "IDLE", 1: "STILL", 2: "6-SIDE", 3: "DONE", 4: "FAILED"}
FACE_BITS = [("+X", 0), ("-X", 1), ("+Y", 2), ("-Y", 3), ("+Z", 4), ("-Z", 5)]


class SerialWorker(QThread):
    telemetry = Signal(object)
    connection = Signal(str, str)
    stats = Signal(object)
    calibration = Signal(object)
    config = Signal(object)
    log_path = Signal(str)
    notice = Signal(str)

    FIELDS = [
        "host_s", "seq", "board_us", "temp_c",
        "ax_mps2", "ay_mps2", "az_mps2",
        "gx_rads", "gy_rads", "gz_rads",
        "roll_deg", "pitch_deg", "yaw_deg",
        "vx_mps", "vy_mps", "vz_mps",
        "px_m", "py_m", "pz_m",
        "lax_mps2", "lay_mps2", "laz_mps2",
        "flags", "nav_status",
        "cal_state", "cal_coverage", "cal_error", "cal_progress",
        "aid_age_ms", "aid_reject",
        "imu_whoami", "imu_class", "observed_sample_hz",
    ]

    def __init__(self, requested="auto", baud=921600, parent=None):
        super().__init__(parent)
        self.requested = requested
        self.baud = baud
        self._stop = False
        self._commands = queue.Queue()
        self._port = None
        self._connected_name = ""
        self._start = time.monotonic()
        self._valid = 0
        self._lost = 0
        self._crc = 0
        self._other = 0
        self._reconnects = 0
        self._last_seq = None
        self._last_emit_stats = 0.0
        self._rate_times = []
        self._log_file = None
        self._writer = None
        self._pending = None

    def stop(self):
        self._stop = True

    def send_cal(self, name):
        if name in CMDS:
            self._commands.put(("cal", name))

    def send_config(self, name, value=None):
        if name in CONFIG_SUB:
            self._commands.put(("config", (name, value)))

    def _open_log(self):
        if self._log_file:
            return
        path = log_output("auto", "gui_session")
        self._log_file = open(path, "w", newline="", buffering=1)
        self._writer = csv.DictWriter(self._log_file, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self.log_path.emit(display_path(path))

    def _write_log(self, d):
        if not self._writer:
            return
        ax, ay, az = (x / 8192.0 * G for x in d["accel_raw"])
        gx, gy, gz = (x / 65.5 * math.pi / 180.0 for x in d["gyro_raw"])
        cs, cc, ce, cp = d["cal"]
        row = dict(
            host_s=time.monotonic() - self._start,
            seq=d["seq"], board_us=d["time_us"], temp_c=d["temp_c"],
            ax_mps2=ax, ay_mps2=ay, az_mps2=az,
            gx_rads=gx, gy_rads=gy, gz_rads=gz,
            roll_deg=d["roll"], pitch_deg=d["pitch"], yaw_deg=d["yaw"],
            vx_mps=d["vel"][0], vy_mps=d["vel"][1], vz_mps=d["vel"][2],
            px_m=d["pos"][0], py_m=d["pos"][1], pz_m=d["pos"][2],
            lax_mps2=d["linacc"][0], lay_mps2=d["linacc"][1], laz_mps2=d["linacc"][2],
            flags=d["flags"], nav_status=d["nav_status"],
            cal_state=cs, cal_coverage=cc, cal_error=ce, cal_progress=cp,
            aid_age_ms=d["aid_age_ms"], aid_reject=d["aid_reject"],
            imu_whoami=d["imu_whoami"], imu_class=d["imu_class"],
            observed_sample_hz=d["observed_sample_hz"],
        )
        self._writer.writerow(row)

    def _close_port(self):
        if self._port is not None:
            try:
                self._port.close()
            except Exception:
                pass
        self._port = None
        self._connected_name = ""

    def _send_cal_packet(self, name):
        self._port.write(cal_packet(bytes((COMM_CAL, CMDS[name]))))
        self._port.flush()

    def _process_commands(self):
        while True:
            try:
                kind, name = self._commands.get_nowait()
            except queue.Empty:
                break
            if kind == "cal" and self._port is not None and self._port.is_open:
                try:
                    self._send_cal_packet(name)
                    self._pending = {
                        "name": name, "sent": time.monotonic(), "attempts": 1
                    }
                    self.notice.emit(f"Command sent: {name}")
                except (serial.SerialException, OSError) as exc:
                    self.notice.emit(f"Command error: {exc}")
                    raise
            elif kind == "config" and self._port is not None and self._port.is_open:
                name, value = name
                sub=CONFIG_SUB[name]
                req=bytes((CONFIG_COMM,sub))
                if name=="output-map": req+=bytes((int(value)&0x0F,))
                try:
                    self._port.write(cfg_packet(req)); self._port.flush()
                    self.notice.emit(f"Config sent: {name}")
                except (serial.SerialException,OSError) as exc:
                    self.notice.emit(f"Config error: {exc}"); raise
        self._retry_pending_if_needed()

    def _confirm_pending(self, ack=None, telemetry=None):
        if not self._pending:
            return
        name = self._pending["name"]
        if ack is not None and ack.get("sub") == CMDS.get(name):
            self.notice.emit(f"Command ACK: {name}")
            self._pending = None
            return
        if telemetry is None:
            return
        state = telemetry["cal"][0]
        confirmed = (
            (name == "still" and state == 1) or
            (name == "rotate-start" and state == 2) or
            (name == "rotate-finish" and state in (3, 4))
        )
        if confirmed:
            self.notice.emit(f"Command confirmed by telemetry: {name}")
            self._pending = None

    def _retry_pending_if_needed(self):
        if not self._pending or self._port is None or not self._port.is_open:
            return
        now = time.monotonic()
        if now - self._pending["sent"] < 0.9:
            return
        name = self._pending["name"]
        if self._pending["attempts"] >= 4:
            self.notice.emit(f"No ACK for {name}; link kept open, telemetry continues")
            self._pending = None
            return
        self._send_cal_packet(name)
        self._pending["attempts"] += 1
        self._pending["sent"] = now
        self.notice.emit(f"Retry {self._pending['attempts']}/4: {name}")

    def _emit_stats(self):
        now = time.monotonic()
        if now - self._last_emit_stats < 0.5:
            return
        self._last_emit_stats = now
        cutoff = now - 2.0
        self._rate_times = [x for x in self._rate_times if x >= cutoff]
        rate = len(self._rate_times) / max(0.2, min(2.0, now - max(cutoff, self._start)))
        self.stats.emit({
            "valid": self._valid,
            "lost": self._lost,
            "crc": self._crc,
            "other": self._other,
            "reconnects": self._reconnects,
            "rate": rate,
        })

    def run(self):
        self._start = time.monotonic()
        try:
            self._open_log()
            while not self._stop:
                try:
                    if self._port is None or not self._port.is_open:
                        self.connection.emit("connecting", f"{self.requested} @ {self.baud}")
                        self._port = open_sideboard_port(
                            self.requested, self.baud, timeout=0.03, attempts=3, delay=0.12
                        )
                        self._port.reset_input_buffer()
                        self._connected_name = self._port.port
                        self.connection.emit("online", self._connected_name)
                        if self._reconnects:
                            self.notice.emit(f"Reconnected: {self._connected_name}")
                    self._process_commands()
                    payload, error = read_frame(self._port)
                    if error:
                        if error == "crc":
                            self._crc += 1
                        elif error != "timeout":
                            self._other += 1
                        self._emit_stats()
                        continue
                    if not payload:
                        continue

                    if payload and payload[0]==CONFIG_COMM and len(payload)>=2:
                        cfg=decode_config(payload,payload[1])
                        if cfg:
                            cfg=dict(cfg); cfg["sub"]=payload[1]
                            self.config.emit(cfg)
                            continue
                    ack = parse_cal_status(payload)
                    if ack:
                        ack = dict(ack)
                        ack["source"] = "ack"
                        self._confirm_pending(ack=ack)
                        self.calibration.emit(ack)
                        continue

                    d = decode_imu(payload)
                    if not d:
                        continue
                    self._valid += 1
                    now = time.monotonic()
                    self._rate_times.append(now)
                    seq = int(d["seq"])
                    if self._last_seq is not None:
                        delta = (seq - self._last_seq) & 0xFFFFFFFF
                        if 1 < delta < 0x80000000:
                            self._lost += delta - 1
                    self._last_seq = seq
                    self._confirm_pending(telemetry=d)
                    self._retry_pending_if_needed()
                    self._write_log(d)
                    self.telemetry.emit(d)
                    self._emit_stats()

                except (serial.SerialException, OSError, FileNotFoundError) as exc:
                    if self._stop:
                        break
                    self._reconnects += 1
                    self.connection.emit("warning", "waiting for serial")
                    self.notice.emit(f"Serial reconnect: {exc}")
                    self._close_port()
                    time.sleep(0.25)
        finally:
            self._close_port()
            if self._log_file:
                try:
                    self._log_file.flush()
                    self._log_file.close()
                except Exception:
                    pass
            self.connection.emit("offline", "disconnected")


def card(title=None):
    f = QFrame()
    f.setObjectName("Card")
    if title is not None:
        layout = QVBoxLayout(f)
        layout.setContentsMargins(14, 12, 14, 14)
        t = QLabel(title)
        t.setObjectName("SectionTitle")
        layout.addWidget(t)
        return f, layout
    return f


def make_step(title, description, button_text, accent=ACCENT):
    frame = QFrame()
    frame.setObjectName("Card")
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(16, 14, 16, 14)
    top = QHBoxLayout()
    dot = QLabel("●")
    dot.setStyleSheet(f"color:{accent}; font-size:14pt;")
    lab = QLabel(title)
    lab.setObjectName("SectionTitle")
    top.addWidget(dot)
    top.addWidget(lab)
    top.addStretch(1)
    lay.addLayout(top)
    desc = QLabel(description)
    desc.setWordWrap(True)
    desc.setObjectName("Muted")
    lay.addWidget(desc)
    btn = QPushButton(button_text)
    btn.setObjectName("Primary")
    lay.addWidget(btn)
    return frame, btn


class ImuDashboard(QMainWindow):
    def __init__(self, auto_connect=True):
        super().__init__()
        self.setWindowTitle("Sideboard IMU Control Center")
        self.resize(1480, 900)
        self.setMinimumSize(1120, 720)
        self.settings = QSettings("Sirobo", "SideboardIMU")
        self.worker = None
        self.latest = None
        self.current_log = ""
        self._last_cal_state = 0
        self._finish_sent = False

        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        outer.addWidget(self._build_sidebar())
        main = QWidget()
        main_lay = QVBoxLayout(main)
        main_lay.setContentsMargins(0, 0, 0, 0)
        main_lay.setSpacing(0)
        main_lay.addWidget(self._build_topbar())
        self.stack = QStackedWidget()
        main_lay.addWidget(self.stack, 1)
        outer.addWidget(main, 1)

        self.pages = [
            self._build_overview(),
            self._build_calibration(),
            self._build_raw(),
            self._build_diagnostics(),
        ]
        for p in self.pages:
            self.stack.addWidget(p)

        self.nav_buttons[0].setChecked(True)
        self._set_page(0)
        self.statusBar().showMessage("Ready · telemetry CSV is saved automatically")

        QShortcut(QKeySequence("Ctrl+1"), self, activated=lambda: self._set_page(0))
        QShortcut(QKeySequence("Ctrl+2"), self, activated=lambda: self._set_page(1))
        QShortcut(QKeySequence("Ctrl+3"), self, activated=lambda: self._set_page(2))
        QShortcut(QKeySequence("Ctrl+4"), self, activated=lambda: self._set_page(3))
        QShortcut(QKeySequence("Ctrl+L"), self, activated=self.open_logs)

        if auto_connect:
            QTimer.singleShot(700, self.connect_serial)

    def _build_sidebar(self):
        side = QFrame()
        side.setObjectName("SideBar")
        side.setFixedWidth(196)
        lay = QVBoxLayout(side)
        lay.setContentsMargins(12, 18, 12, 14)
        lay.setSpacing(5)

        brand = QLabel("SIDEBOARD")
        brand.setStyleSheet("font-size:15pt; font-weight:800; letter-spacing:2px; color:#e8f0f8;")
        sub = QLabel("IMU CONTROL CENTER")
        sub.setObjectName("Muted")
        sub.setStyleSheet("font-size:8pt; letter-spacing:1px;")
        lay.addWidget(brand)
        lay.addWidget(sub)
        lay.addSpacing(18)

        names = [("◈  Overview", 0), ("◎  Calibration", 1), ("⌁  Raw Sensors", 2), ("≡  Diagnostics", 3)]
        self.nav_buttons = []
        for text, idx in names:
            b = QPushButton(text)
            b.setObjectName("Nav")
            b.setCheckable(True)
            b.clicked.connect(lambda checked=False, i=idx: self._set_page(i))
            lay.addWidget(b)
            self.nav_buttons.append(b)
        lay.addStretch(1)

        self.side_log = QLabel("CSV\nauto-save enabled")
        self.side_log.setObjectName("Muted")
        self.side_log.setWordWrap(True)
        lay.addWidget(self.side_log)

        open_btn = QPushButton("Open logs folder")
        open_btn.clicked.connect(self.open_logs)
        lay.addWidget(open_btn)
        return side

    def _build_topbar(self):
        top = QFrame()
        top.setObjectName("TopBar")
        top.setFixedHeight(70)
        lay = QHBoxLayout(top)
        lay.setContentsMargins(18, 10, 18, 10)

        title_box = QVBoxLayout()
        self.page_title = QLabel("Overview")
        self.page_title.setObjectName("Title")
        self.page_subtitle = QLabel("Live attitude, navigation and health")
        self.page_subtitle.setObjectName("Muted")
        title_box.addWidget(self.page_title)
        title_box.addWidget(self.page_subtitle)
        lay.addLayout(title_box)
        lay.addStretch(1)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(130)
        self.refresh_ports()
        saved = self.settings.value("port", "auto")
        self.port_combo.setCurrentText(str(saved))
        lay.addWidget(self.port_combo)

        refresh = QPushButton("↻")
        refresh.setFixedWidth(38)
        refresh.setToolTip("Refresh serial ports")
        refresh.clicked.connect(self.refresh_ports)
        lay.addWidget(refresh)

        self.conn_pill = StatusPill("OFFLINE")
        lay.addWidget(self.conn_pill)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("Primary")
        self.connect_btn.clicked.connect(self.toggle_connection)
        lay.addWidget(self.connect_btn)
        return top

    def _scroll_page(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        body = QWidget()
        lay = QVBoxLayout(body)
        lay.setContentsMargins(18, 18, 18, 22)
        lay.setSpacing(14)
        scroll.setWidget(body)
        return scroll, lay

    def _build_overview(self):
        page, lay = self._scroll_page()

        attitude_card = QFrame(); attitude_card.setObjectName("Card")
        al = QHBoxLayout(attitude_card); al.setContentsMargins(12, 12, 12, 12)
        self.body3d = Attitude3DWidget()
        self.horizon = ArtificialHorizonWidget()
        al.addWidget(self.body3d, 3)
        al.addWidget(self.horizon, 2)
        lay.addWidget(attitude_card, 3)

        grid = QGridLayout()
        grid.setSpacing(10)
        self.m_roll = MetricCard("ROLL", "°", "#ff6b6b")
        self.m_pitch = MetricCard("PITCH", "°", "#69db7c")
        self.m_yaw = MetricCard("YAW", "°", "#74c0fc")
        self.m_speed = MetricCard("SPEED", "m/s", ACCENT_2)
        self.m_position = MetricCard("POSITION", "m", WARN)
        self.m_temp = MetricCard("IMU TEMP", "°C", "#ff922b")
        for i, w in enumerate([self.m_roll, self.m_pitch, self.m_yaw, self.m_speed, self.m_position, self.m_temp]):
            grid.addWidget(w, i // 3, i % 3)
        lay.addLayout(grid)

        self.plot_rpy = LivePlotWidget(
            "ATTITUDE HISTORY", [("R", "#ff6b6b"), ("P", "#69db7c"), ("Y", "#74c0fc")],
            max_points=750, fixed_range=(-180, 180)
        )
        self.plot_vel = LivePlotWidget(
            "VELOCITY HISTORY", [("VX", "#ff6b6b"), ("VY", "#69db7c"), ("VZ", "#74c0fc")],
            max_points=750
        )
        plots = QHBoxLayout(); plots.setSpacing(10)
        plots.addWidget(self.plot_rpy)
        plots.addWidget(self.plot_vel)
        lay.addLayout(plots)
        return page

    def _build_calibration(self):
        page, lay = self._scroll_page()

        status, sl = card("Calibration State")
        row = QHBoxLayout()
        self.cal_pill = StatusPill("IDLE")
        self.cal_text = QLabel("Waiting for telemetry")
        self.cal_text.setObjectName("Muted")
        row.addWidget(self.cal_pill)
        row.addWidget(self.cal_text)
        row.addStretch(1)
        sl.addLayout(row)
        self.cal_progress = QProgressBar()
        self.cal_progress.setRange(0, 1000)
        self.cal_progress.setValue(0)
        sl.addWidget(self.cal_progress)
        lay.addWidget(status)

        steps = QGridLayout(); steps.setSpacing(12)
        still, self.btn_still = make_step(
            "1 · STILL calibration",
            "Keep the board completely still. Captures gyro bias and reference temperature.",
            "Start STILL"
        )
        six, self.btn_six = make_step(
            "2 · Accelerometer 6-side",
            "Hold +X, -X, +Y, -Y, +Z and -Z. Coverage is tracked live.",
            "Start 6-side", ACCENT_2
        )
        steps.addWidget(still, 0, 0)
        steps.addWidget(six, 0, 1)
        lay.addLayout(steps)

        cov, cl = card("6-side Coverage")
        self.face_labels = {}
        face_row = QHBoxLayout()
        for label, bit in FACE_BITS:
            x = QLabel(label)
            x.setAlignment(Qt.AlignmentFlag.AlignCenter)
            x.setMinimumHeight(38)
            self.face_labels[bit] = x
            face_row.addWidget(x)
        cl.addLayout(face_row)
        self._update_coverage(0)
        lay.addWidget(cov)

        controls, ctl = card("Navigation / Stationary Controls")
        buttons = QGridLayout()
        self.btn_zero = QPushButton("ZERO NAV")
        self.btn_zero.setObjectName("Primary")
        self.btn_zupt = QPushButton("ZUPT once")
        self.btn_on = QPushButton("Stationary ON")
        self.btn_off = QPushButton("Stationary OFF")
        self.btn_off.setObjectName("Danger")
        self.btn_cancel = QPushButton("Cancel calibration")
        for i, b in enumerate([self.btn_zero, self.btn_zupt, self.btn_on, self.btn_off, self.btn_cancel]):
            buttons.addWidget(b, i // 3, i % 3)
        ctl.addLayout(buttons)
        hint = QLabel(
            "Tip: for AGV operation, wheel/world-velocity aiding should control motion. "
            "STATIONARY_OFF prevents automatic rest re-arm during commanded motion."
        )
        hint.setWordWrap(True); hint.setObjectName("Muted")
        ctl.addWidget(hint)
        lay.addWidget(controls)

        mapping, ml = card("Axis / Output Mapping")
        mrow=QHBoxLayout()
        self.chk_ix=QCheckBox("Invert X")
        self.chk_iy=QCheckBox("Invert Y")
        self.chk_iz=QCheckBox("Invert Z")
        self.chk_swap=QCheckBox("Swap Roll / Pitch")
        for w in (self.chk_ix,self.chk_iy,self.chk_iz,self.chk_swap): mrow.addWidget(w)
        mrow.addStretch(1)
        ml.addLayout(mrow)
        brow=QHBoxLayout()
        self.btn_map_apply=QPushButton("Apply Mapping"); self.btn_map_apply.setObjectName("Primary")
        self.btn_map_read=QPushButton("Refresh")
        self.btn_map_reset=QPushButton("Reset Mapping")
        self.btn_reset_all=QPushButton("RESET ALL"); self.btn_reset_all.setObjectName("Danger")
        for w in (self.btn_map_apply,self.btn_map_read,self.btn_map_reset,self.btn_reset_all): brow.addWidget(w)
        brow.addStretch(1); ml.addLayout(brow)
        self.map_status=QLabel("Mapping belum dibaca"); self.map_status.setObjectName("Muted"); ml.addWidget(self.map_status)
        warn=QLabel("Reset All menghapus calibration STILL, 6-side, thermal, mount, noise tuning, lever arm, dan mapping.")
        warn.setWordWrap(True); warn.setObjectName("Muted"); ml.addWidget(warn)
        lay.addWidget(mapping)

        self.btn_map_apply.clicked.connect(self.apply_output_map)
        self.btn_map_read.clicked.connect(lambda: self.send_config("get"))
        self.btn_map_reset.clicked.connect(lambda: self.send_config("output-map",0))
        self.btn_reset_all.clicked.connect(self.reset_all_confirm)

        self.btn_still.clicked.connect(lambda: self.send_command("still"))
        self.btn_six.clicked.connect(lambda: self._start_six())
        self.btn_zero.clicked.connect(lambda: self.send_command("zero-nav"))
        self.btn_zupt.clicked.connect(lambda: self.send_command("zupt"))
        self.btn_on.clicked.connect(lambda: self.send_command("stationary-on"))
        self.btn_off.clicked.connect(lambda: self.send_command("stationary-off"))
        self.btn_cancel.clicked.connect(lambda: self.send_command("cancel"))
        return page

    def _build_raw(self):
        page, lay = self._scroll_page()

        self.raw_accel = VectorBar("RAW ACCELERATION · m/s²")
        self.raw_gyro = VectorBar("RAW GYRO · deg/s")
        bars = QHBoxLayout(); bars.setSpacing(10)
        bars.addWidget(self.raw_accel); bars.addWidget(self.raw_gyro)
        lay.addLayout(bars)

        self.plot_acc = LivePlotWidget(
            "RAW ACCEL", [("AX", "#ff6b6b"), ("AY", "#69db7c"), ("AZ", "#74c0fc")], max_points=600
        )
        self.plot_gyro = LivePlotWidget(
            "RAW GYRO", [("GX", "#ff6b6b"), ("GY", "#69db7c"), ("GZ", "#74c0fc")], max_points=600
        )
        lay.addWidget(self.plot_acc)
        lay.addWidget(self.plot_gyro)

        table_card, tl = card("Current Raw / Processed Values")
        self.raw_table = QTableWidget(0, 3)
        self.raw_table.setHorizontalHeaderLabels(["Signal", "Value", "Unit"])
        self.raw_table.horizontalHeader().setStretchLastSection(True)
        self.raw_table.verticalHeader().setVisible(False)
        self.raw_table.setAlternatingRowColors(True)
        tl.addWidget(self.raw_table)
        lay.addWidget(table_card)
        return page

    def _build_diagnostics(self):
        page, lay = self._scroll_page()

        grid = QGridLayout(); grid.setSpacing(10)
        self.d_rate = MetricCard("HOST STREAM", "Hz", ACCENT)
        self.d_sample = MetricCard("SENSOR RATE", "Hz", ACCENT_2)
        self.d_lost = MetricCard("LOST FRAMES", "", WARN)
        self.d_crc = MetricCard("CRC ERRORS", "", DANGER)
        self.d_reset = MetricCard("ESKF RESETS", "", DANGER)
        self.d_aid = MetricCard("AID AGE", "ms", "#b197fc")
        for i, w in enumerate([self.d_rate, self.d_sample, self.d_lost, self.d_crc, self.d_reset, self.d_aid]):
            grid.addWidget(w, i // 3, i % 3)
        lay.addLayout(grid)

        health, hl = card("Health / Flags")
        self.health_table = QTableWidget(0, 2)
        self.health_table.setHorizontalHeaderLabels(["Item", "State"])
        self.health_table.horizontalHeader().setStretchLastSection(True)
        self.health_table.verticalHeader().setVisible(False)
        hl.addWidget(self.health_table)
        lay.addWidget(health)

        log_card, ll = card("Session Logging")
        self.log_label = QLabel("Waiting for session log")
        self.log_label.setWordWrap(True)
        self.log_label.setObjectName("Muted")
        ll.addWidget(self.log_label)
        b = QPushButton("Open logs folder  ·  Ctrl+L")
        b.clicked.connect(self.open_logs)
        ll.addWidget(b)
        lay.addWidget(log_card)
        return page

    def _set_page(self, index):
        titles = [
            ("Overview", "Live attitude, navigation and health"),
            ("Calibration", "Guided sensor calibration and zero-velocity controls"),
            ("Raw Sensors", "Unfiltered sensor stream and processed values"),
            ("Diagnostics", "Protocol health, aiding state and automatic session logs"),
        ]
        if hasattr(self, "stack"):
            self.stack.setCurrentIndex(index)
        if hasattr(self, "page_title"):
            self.page_title.setText(titles[index][0])
            self.page_subtitle.setText(titles[index][1])
        if hasattr(self, "nav_buttons"):
            for i, b in enumerate(self.nav_buttons):
                b.setChecked(i == index)

    def refresh_ports(self):
        current = self.port_combo.currentText() if hasattr(self, "port_combo") else "auto"
        if hasattr(self, "port_combo"):
            self.port_combo.clear()
            self.port_combo.addItem("auto")
            for p in list_ports.comports():
                self.port_combo.addItem(p.device)
            self.port_combo.setCurrentText(current or "auto")

    def toggle_connection(self):
        if self.worker and self.worker.isRunning():
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        if self.worker and self.worker.isRunning():
            return
        port = self.port_combo.currentText().strip() or "auto"
        self.settings.setValue("port", port)
        self.worker = SerialWorker(port, 921600, self)
        self.worker.telemetry.connect(self.on_telemetry)
        self.worker.connection.connect(self.on_connection)
        self.worker.stats.connect(self.on_stats)
        self.worker.calibration.connect(self.on_calibration)
        self.worker.config.connect(self.on_config)
        self.worker.log_path.connect(self.on_log_path)
        self.worker.notice.connect(lambda s: self.statusBar().showMessage(s, 4500))
        self.worker.start()
        self.connect_btn.setText("Disconnect")

    def disconnect_serial(self):
        if self.worker:
            self.worker.stop()
            self.worker.wait(2500)
            self.worker = None
        self.connect_btn.setText("Connect")
        self.conn_pill.set_state("offline", "OFFLINE")

    def on_connection(self, state, detail):
        mapping = {
            "online": ("online", "ONLINE"),
            "connecting": ("busy", "CONNECTING"),
            "warning": ("warning", "RECONNECT"),
            "offline": ("offline", "OFFLINE"),
        }
        pill, text = mapping.get(state, ("offline", state.upper()))
        self.conn_pill.set_state(pill, text)
        self.conn_pill.setToolTip(detail)
        if state == "online":
            self.connect_btn.setText("Disconnect")
            self.statusBar().showMessage(f"Connected to {detail}", 3500)
            if self.worker: QTimer.singleShot(250, lambda: self.worker and self.worker.send_config("get"))
        elif state == "offline":
            self.connect_btn.setText("Connect")


    def current_output_map(self):
        return ((1 if self.chk_ix.isChecked() else 0) |
                (2 if self.chk_iy.isChecked() else 0) |
                (4 if self.chk_iz.isChecked() else 0) |
                (8 if self.chk_swap.isChecked() else 0))

    def send_config(self,name,value=None):
        if not self.worker or not self.worker.isRunning():
            QMessageBox.warning(self,"Not connected","Connect to the sideboard first.")
            return
        self.worker.send_config(name,value)
        self.statusBar().showMessage(f"Queued config: {name}",2500)

    def apply_output_map(self):
        self.send_config("output-map",self.current_output_map())

    def reset_all_confirm(self):
        ans=QMessageBox.warning(
            self,"RESET ALL IMU",
            "This deletes ALL calibration and IMU configuration from EEPROM.\n\n"
            "STILL, 6-side, thermal, mount, noise tuning, lever arm and axis mapping will be reset.\n\n"
            "Continue?",
            QMessageBox.StandardButton.Yes|QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel)
        if ans!=QMessageBox.StandardButton.Yes:return
        self.send_config("reset-all")

    def on_config(self,d):
        status=int(d.get("status",255))
        if "output_map" in d:
            m=int(d.get("output_map",0))
            self.chk_ix.setChecked(bool(m&1)); self.chk_iy.setChecked(bool(m&2))
            self.chk_iz.setChecked(bool(m&4)); self.chk_swap.setChecked(bool(m&8))
            self.map_status.setText(
                f"EEPROM map 0x{m:02X} · X {'INV' if m&1 else 'normal'} · "
                f"Y {'INV' if m&2 else 'normal'} · Z {'INV' if m&4 else 'normal'} · "
                f"Roll/Pitch {'SWAPPED' if m&8 else 'normal'}")
        self.statusBar().showMessage(f"Config reply sub={d.get('sub')} status={status}",3500)
        if d.get("sub")==CONFIG_SUB.get("reset-all") and status==0:
            QMessageBox.information(self,"Reset All","Reset All selesai. Jalankan kalibrasi STILL dan 6-side kembali.")

    def on_log_path(self, path):
        self.current_log = path
        self.side_log.setText(f"CSV auto-save\n{Path(path).name}")
        self.log_label.setText(path)

    def on_stats(self, s):
        self.d_rate.set_value(f"{s['rate']:.1f}", f"{s['valid']} valid frames")
        self.d_lost.set_value(str(s["lost"]), f"reconnects {s['reconnects']}")
        self.d_crc.set_value(str(s["crc"]), f"other parser errors {s['other']}")

    def on_calibration(self, st):
        self._update_calibration(
            st.get("state", 0), st.get("coverage", 0), st.get("error", 0),
            int(round(float(st.get("progress", 0)) * 10))
        )
        self.statusBar().showMessage(
            f"Calibration ACK · {CAL_STATE.get(st.get('state', 0), '?')} · status={st.get('status', 0)}",
            3500,
        )

    def _update_calibration(self, state, coverage, error, progress):
        self._last_cal_state = state
        name = CAL_STATE.get(state, f"STATE {state}")
        if state == 4 or error:
            self.cal_pill.set_state("danger", name)
        elif state in (1, 2):
            self.cal_pill.set_state("busy", name)
        elif state == 3:
            self.cal_pill.set_state("online", name)
        else:
            self.cal_pill.set_state("offline", name)
        self.cal_progress.setValue(max(0, min(1000, int(progress))))
        self.cal_text.setText(
            f"progress {progress/10:.1f}% · coverage 0x{coverage:02X} · error {error}"
        )
        self._update_coverage(coverage)

        if state == 2 and coverage == 0x3F and not self._finish_sent:
            self._finish_sent = True
            QTimer.singleShot(350, lambda: self.send_command("rotate-finish"))
        if state != 2:
            self._finish_sent = False

    def _update_coverage(self, coverage):
        for label, bit in FACE_BITS:
            hit = bool(coverage & (1 << bit))
            fg = "#102218" if hit else "#8ea0b5"
            bg = GOOD if hit else "#17212d"
            border = GOOD if hit else "#2b3a4d"
            self.face_labels[bit].setStyleSheet(
                f"background:{bg}; color:{fg}; border:1px solid {border};"
                "border-radius:9px; font-weight:800;"
            )

    def _start_six(self):
        self._finish_sent = False
        self.send_command("rotate-start")

    def send_command(self, name):
        if not self.worker or not self.worker.isRunning():
            QMessageBox.warning(self, "Not connected", "Connect to the sideboard first.")
            return
        self.worker.send_cal(name)
        self.statusBar().showMessage(f"Queued command: {name}", 2500)

    def on_telemetry(self, d):
        self.latest = d
        r, p, y = d["roll"], d["pitch"], d["yaw"]
        vx, vy, vz = d["vel"]
        px, py, pz = d["pos"]
        lax, lay, laz = d["linacc"]
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        pos_norm = math.sqrt(px*px + py*py + pz*pz)

        self.body3d.set_attitude(r, p, y)
        self.horizon.set_attitude(r, p, y)
        self.plot_rpy.add_values(r, p, y)
        self.plot_vel.add_values(vx, vy, vz)

        self.m_roll.set_value(f"{r:+.2f}", f"σ {d['att_std_deg'][0]:.2f}°")
        self.m_pitch.set_value(f"{p:+.2f}", f"σ {d['att_std_deg'][1]:.2f}°")
        self.m_yaw.set_value(f"{y%360:6.2f}", f"σ {d['att_std_deg'][2]:.2f}°")
        self.m_speed.set_value(f"{speed:.3f}", f"vx {vx:+.3f} · vy {vy:+.3f} · vz {vz:+.3f}")
        self.m_position.set_value(f"{pos_norm:.3f}", f"x {px:+.3f} · y {py:+.3f} · z {pz:+.3f}")
        self.m_temp.set_value(f"{d['temp_c']:.1f}", f"WHOAMI 0x{d['imu_whoami']:02X} · class {d['imu_class']}")

        ax, ay, az = (x / 8192.0 * G for x in d["accel_raw"])
        gx, gy, gz = (x / 65.5 for x in d["gyro_raw"])
        self.raw_accel.set_values(ax, ay, az)
        self.raw_gyro.set_values(gx, gy, gz)
        self.plot_acc.add_values(ax, ay, az)
        self.plot_gyro.add_values(gx, gy, gz)

        cs, cc, ce, cp = d["cal"]
        self._update_calibration(cs, cc, ce, cp)

        self.d_sample.set_value(f"{d['observed_sample_hz']:.1f}", "sensor measured rate")
        self.d_reset.set_value(str(d["health_resets"]), "ESKF health recoveries")
        age = d["aid_age_ms"]
        self.d_aid.set_value("—" if age >= 65535 else str(age), f"rejected {d['aid_reject']}")

        self._update_raw_table(d, (ax, ay, az), (gx, gy, gz))
        self._update_health_table(d)

    def _update_raw_table(self, d, acc, gyro):
        rows = [
            ("Accel X", f"{acc[0]:+.5f}", "m/s²"),
            ("Accel Y", f"{acc[1]:+.5f}", "m/s²"),
            ("Accel Z", f"{acc[2]:+.5f}", "m/s²"),
            ("Gyro X", f"{gyro[0]:+.4f}", "deg/s"),
            ("Gyro Y", f"{gyro[1]:+.4f}", "deg/s"),
            ("Gyro Z", f"{gyro[2]:+.4f}", "deg/s"),
            ("Linear Acc X", f"{d['linacc'][0]:+.4f}", "m/s²"),
            ("Linear Acc Y", f"{d['linacc'][1]:+.4f}", "m/s²"),
            ("Linear Acc Z", f"{d['linacc'][2]:+.4f}", "m/s²"),
            ("Board time", str(d["time_us"]), "µs"),
            ("Sequence", str(d["seq"]), ""),
        ]
        self.raw_table.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col == 1:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.raw_table.setItem(row, col, item)
        self.raw_table.resizeColumnsToContents()

    def _update_health_table(self, d):
        flags = d["flags"]
        items = [
            ("Sensor", "OK" if flags & 1 else "FAULT"),
            ("ESKF", "OK" if flags & 2 else "FAULT"),
            ("EEPROM", "VALID" if flags & 8 else "INVALID"),
            ("Still calibration", "VALID" if flags & (1 << 6) else "MISSING"),
            ("6-side calibration", "VALID" if flags & (1 << 7) else "MISSING"),
            ("ZUPT", "ACTIVE" if flags & (1 << 9) else "OFF"),
            ("Master stationary", "ON" if flags & (1 << 10) else "OFF"),
            ("Wheel aiding", "ACTIVE" if flags & (1 << 11) else "OFF"),
            ("NHC", "ACTIVE" if flags & (1 << 12) else "OFF"),
            ("Yaw aiding", "ACTIVE" if flags & (1 << 13) else "OFF"),
            ("Velocity aiding", "ACTIVE" if flags & (1 << 14) else "OFF"),
            ("Position aiding", "ACTIVE" if flags & (1 << 15) else "OFF"),
            ("Navigation", nav_text(d["nav_status"]) or "none"),
        ]
        self.health_table.setRowCount(len(items))
        for row, (k, v) in enumerate(items):
            self.health_table.setItem(row, 0, QTableWidgetItem(k))
            item = QTableWidgetItem(v)
            if v in ("OK", "VALID", "ACTIVE", "ON"):
                item.setForeground(QColor(GOOD))
            elif v in ("FAULT", "INVALID", "MISSING"):
                item.setForeground(QColor(DANGER))
            else:
                item.setForeground(QColor(MUTED))
            self.health_table.setItem(row, 1, item)

    def open_logs(self):
        path = Path(LOG_DIR)
        path.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path.resolve())))

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("Sideboard IMU Control Center")
    app.setOrganizationName("Sirobo")
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    win = ImuDashboard()
    win.show()
    return app.exec()


if __name__ == "__main__":
    from run_csv import run_logged
    raise SystemExit(run_logged(main, __file__))
