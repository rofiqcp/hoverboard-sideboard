"""Native Qt6 widgets for the sideboard IMU dashboard.

No QtCharts/QML/OpenGL dependency: all high-rate visuals use QWidget + QPainter.
"""
import math
from collections import deque

from PySide6.QtCore import Qt, QTimer, QPointF, QRectF
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QBrush, QLinearGradient
from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QSizePolicy

from gui_theme import ACCENT, ACCENT_2, BORDER, DANGER, GOOD, MUTED, PANEL, TEXT, WARN


def _angle_lerp(current, target, alpha):
    delta = (target - current + 180.0) % 360.0 - 180.0
    return current + delta * alpha


class StatusPill(QLabel):
    def __init__(self, text="OFFLINE", parent=None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumWidth(84)
        self.set_state("offline", text)

    def set_state(self, state, text=None):
        palette = {
            "online": (GOOD, "#10251a"),
            "warning": (WARN, "#2b2616"),
            "danger": (DANGER, "#2f171c"),
            "offline": (MUTED, "#18202a"),
            "busy": (ACCENT, "#10273a"),
        }
        fg, bg = palette.get(state, palette["offline"])
        if text is not None:
            self.setText(text)
        self.setStyleSheet(
            f"QLabel {{ color:{fg}; background:{bg}; border:1px solid {fg};"
            "border-radius:10px; padding:4px 9px; font-weight:700; }"
        )


class MetricCard(QFrame):
    def __init__(self, title, unit="", accent=ACCENT, parent=None):
        super().__init__(parent)
        self.setObjectName("Card")
        self.setMinimumHeight(96)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 11, 14, 11)
        layout.setSpacing(2)

        top = QHBoxLayout()
        self.title = QLabel(title)
        self.title.setObjectName("Muted")
        dot = QLabel("●")
        dot.setStyleSheet(f"color:{accent}; font-size:9pt;")
        top.addWidget(self.title)
        top.addStretch(1)
        top.addWidget(dot)
        layout.addLayout(top)

        row = QHBoxLayout()
        self.value = QLabel("—")
        self.value.setObjectName("MetricValue")
        self.unit = QLabel(unit)
        self.unit.setObjectName("Muted")
        row.addWidget(self.value)
        row.addWidget(self.unit, 0, Qt.AlignmentFlag.AlignBottom)
        row.addStretch(1)
        layout.addLayout(row)

        self.detail = QLabel("waiting for telemetry")
        self.detail.setObjectName("Muted")
        self.detail.setStyleSheet("font-size:8.5pt;")
        layout.addWidget(self.detail)

    def set_value(self, value, detail=None):
        self.value.setText(str(value))
        if detail is not None:
            self.detail.setText(detail)


class Attitude3DWidget(QWidget):
    """Lightweight perspective body model driven by roll/pitch/yaw."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 260)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.target = [0.0, 0.0, 0.0]
        self.current = [0.0, 0.0, 0.0]
        self.view_yaw = -18.0
        self.view_pitch = 12.0
        self.zoom = 1.0
        self._drag_pos = None
        self.setToolTip("Drag: orbit view · Mouse wheel: zoom · Double-click: reset camera")
        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._animate)
        self._timer.start()

    def set_attitude(self, roll, pitch, yaw):
        self.target = [float(roll), float(pitch), float(yaw)]

    def _animate(self):
        self.current[0] = _angle_lerp(self.current[0], self.target[0], 0.18)
        self.current[1] = _angle_lerp(self.current[1], self.target[1], 0.18)
        self.current[2] = _angle_lerp(self.current[2], self.target[2], 0.13)
        self.update()

    @staticmethod
    def _rotate(v, roll, pitch, yaw):
        x, y, z = v
        r, p, h = map(math.radians, (roll, pitch, yaw))
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        ch, sh = math.cos(h), math.sin(h)

        # body roll X
        y, z = cr * y - sr * z, sr * y + cr * z
        # pitch Y
        x, z = cp * x + sp * z, -sp * x + cp * z
        # yaw Z
        x, y = ch * x - sh * y, sh * x + ch * y
        return x, y, z

    def _camera(self, v):
        x, y, z = v
        a = math.radians(self.view_yaw)
        ca, sa = math.cos(a), math.sin(a)
        x, y = ca * x - sa * y, sa * x + ca * y
        b = math.radians(self.view_pitch)
        cb, sb = math.cos(b), math.sin(b)
        y, z = cb * y - sb * z, sb * y + cb * z
        return x, y, z

    def _project(self, v):
        x, y, z = self._camera(v)
        depth = 6.5 - y
        scale = min(self.width(), self.height()) * 0.18 * self.zoom / max(2.5, depth)
        cx, cy = self.width() * 0.50, self.height() * 0.49
        return QPointF(cx + x * scale * 5.0, cy - z * scale * 5.0)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_pos = event.position()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._drag_pos is None:
            return
        pos = event.position()
        delta = pos - self._drag_pos
        self._drag_pos = pos
        self.view_yaw += delta.x() * 0.45
        self.view_pitch = max(-70.0, min(70.0, self.view_pitch + delta.y() * 0.35))
        self.update()

    def mouseReleaseEvent(self, event):
        self._drag_pos = None
        self.unsetCursor()

    def wheelEvent(self, event):
        self.zoom *= 1.0 + (0.10 if event.angleDelta().y() > 0 else -0.10)
        self.zoom = max(0.65, min(1.8, self.zoom))
        self.update()

    def mouseDoubleClickEvent(self, event):
        self.view_yaw, self.view_pitch, self.zoom = -18.0, 12.0, 1.0
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0e1721"))

        # soft radial-ish backdrop
        grad = QLinearGradient(0, 0, self.width(), self.height())
        grad.setColorAt(0, QColor("#132438"))
        grad.setColorAt(0.55, QColor("#0d1721"))
        grad.setColorAt(1, QColor("#0a1118"))
        p.fillRect(self.rect(), grad)

        # reference rings
        c = QPointF(self.width() / 2, self.height() / 2)
        p.setPen(QPen(QColor("#26384b"), 1))
        for factor in (0.28, 0.43):
            rr = min(self.width(), self.height()) * factor
            p.drawEllipse(c, rr, rr)

        # body: x forward (nose), y left/right, z up
        verts = [
            (1.7, -0.72, -0.28), (1.7, 0.72, -0.28),
            (-1.25, -0.72, -0.28), (-1.25, 0.72, -0.28),
            (1.7, -0.72, 0.28), (1.7, 0.72, 0.28),
            (-1.25, -0.72, 0.28), (-1.25, 0.72, 0.28),
            (2.45, 0.0, 0.0),
        ]
        rv = [self._rotate(v, *self.current) for v in verts]
        pts = [self._project(v) for v in rv]

        # sort faces by average depth
        faces = [
            ((0, 1, 5, 4), "#1c86c8"),
            ((2, 3, 7, 6), "#12334a"),
            ((0, 2, 6, 4), "#174e70"),
            ((1, 3, 7, 5), "#206c94"),
            ((4, 5, 7, 6), "#2b9fca"),
            ((0, 1, 3, 2), "#103047"),
            ((0, 4, 8), "#41c7ff"),
            ((1, 5, 8), "#299ecf"),
        ]
        ordered = sorted(
            faces,
            key=lambda fc: sum(rv[i][1] for i in fc[0]) / len(fc[0]),
            reverse=True,
        )
        for ids, color in ordered:
            path = QPainterPath()
            path.moveTo(pts[ids[0]])
            for i in ids[1:]:
                path.lineTo(pts[i])
            path.closeSubpath()
            p.setBrush(QBrush(QColor(color)))
            p.setPen(QPen(QColor("#80d9ff"), 1.1))
            p.drawPath(path)

        # axes
        origin = self._project(self._rotate((0, 0, 0), *self.current))
        axes = [
            ((2.9, 0, 0), QColor("#ff6b6b"), "X"),
            ((0, 2.3, 0), QColor("#69db7c"), "Y"),
            ((0, 0, 2.0), QColor("#74c0fc"), "Z"),
        ]
        font = QFont(self.font())
        font.setPointSize(8)
        font.setBold(True)
        p.setFont(font)
        for vec, color, label in axes:
            end = self._project(self._rotate(vec, *self.current))
            p.setPen(QPen(color, 2))
            p.drawLine(origin, end)
            p.drawText(end + QPointF(5, -3), label)

        # heading/attitude labels
        p.setPen(QColor(TEXT))
        font.setPointSize(10)
        p.setFont(font)
        p.drawText(14, 24, "3D BODY FRAME")
        p.setPen(QColor(MUTED))
        p.drawText(
            14, self.height() - 15,
            f"R {self.current[0]:+6.1f}°   P {self.current[1]:+6.1f}°   Y {self.current[2]%360:6.1f}°"
        )


class ArtificialHorizonWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = self.pitch = self.yaw = 0.0
        self.setMinimumSize(250, 250)

    def set_attitude(self, roll, pitch, yaw):
        self.roll, self.pitch, self.yaw = float(roll), float(pitch), float(yaw)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        side = min(w, h) - 18
        cx, cy = w / 2, h / 2
        radius = side / 2

        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), radius, radius)
        p.save()
        p.setClipPath(clip)
        p.translate(cx, cy)
        p.rotate(-self.roll)
        pitch_px = self.pitch * (side / 90.0)
        p.translate(0, pitch_px)

        sky = QColor("#2377b8")
        ground = QColor("#805a32")
        p.fillRect(QRectF(-side, -side * 2, side * 2, side * 2), sky)
        p.fillRect(QRectF(-side, 0, side * 2, side * 2), ground)

        p.setPen(QPen(QColor("#f4f7fa"), 2))
        p.drawLine(QPointF(-side, 0), QPointF(side, 0))

        # pitch ladder
        font = QFont(self.font())
        font.setPointSize(7)
        p.setFont(font)
        for deg in range(-40, 50, 10):
            if deg == 0:
                continue
            y = -deg * (side / 90.0)
            length = side * (0.17 if deg % 20 else 0.25)
            p.setPen(QPen(QColor(255, 255, 255, 205), 1.2))
            p.drawLine(QPointF(-length, y), QPointF(length, y))
            p.drawText(QPointF(length + 5, y + 3), str(abs(deg)))
            p.drawText(QPointF(-length - 20, y + 3), str(abs(deg)))
        p.restore()

        # bezel
        p.setPen(QPen(QColor("#6d8298"), 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        # fixed aircraft symbol
        p.setPen(QPen(QColor("#ffd43b"), 3))
        p.drawLine(QPointF(cx - side * .18, cy), QPointF(cx - side * .045, cy))
        p.drawLine(QPointF(cx + side * .045, cy), QPointF(cx + side * .18, cy))
        p.drawLine(QPointF(cx - side * .045, cy), QPointF(cx, cy + 7))
        p.drawLine(QPointF(cx, cy + 7), QPointF(cx + side * .045, cy))

        # roll ticks
        p.setPen(QPen(QColor("#dbe7f4"), 1.2))
        for deg in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            a = math.radians(deg - 90)
            r1 = radius - (11 if deg % 30 == 0 else 7)
            r2 = radius - 2
            p.drawLine(
                QPointF(cx + math.cos(a) * r1, cy + math.sin(a) * r1),
                QPointF(cx + math.cos(a) * r2, cy + math.sin(a) * r2),
            )

        # heading badge
        heading = self.yaw % 360.0
        rect = QRectF(cx - 34, cy - radius - 3, 68, 23)
        p.setBrush(QColor("#0c1721"))
        p.setPen(QPen(QColor(ACCENT), 1))
        p.drawRoundedRect(rect, 7, 7)
        p.setPen(QColor(TEXT))
        f = QFont(self.font())
        f.setBold(True)
        p.setFont(f)
        p.drawText(rect, Qt.AlignmentFlag.AlignCenter, f"{heading:05.1f}°")


class LivePlotWidget(QWidget):
    """Small rolling plot optimized for 10-20 Hz repaints."""

    def __init__(self, title, series, max_points=600, fixed_range=None, parent=None):
        super().__init__(parent)
        self.title = title
        self.series = series  # [(label, color), ...]
        self.hist = [deque(maxlen=max_points) for _ in series]
        self.fixed_range = fixed_range
        self.setMinimumHeight(180)
        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self.update)
        self._timer.start()

    def add_values(self, *values):
        for hist, value in zip(self.hist, values):
            try:
                hist.append(float(value))
            except (TypeError, ValueError):
                hist.append(float("nan"))

    def clear(self):
        for h in self.hist:
            h.clear()
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0f1721"))
        w, h = self.width(), self.height()
        left, right, top, bottom = 48, 12, 33, 24
        plot = QRectF(left, top, max(10, w - left - right), max(10, h - top - bottom))

        p.setPen(QColor(TEXT))
        f = QFont(self.font())
        f.setBold(True)
        p.setFont(f)
        p.drawText(12, 21, self.title)

        # legend
        x = max(130, w - 240)
        f.setPointSize(8)
        p.setFont(f)
        for i, (name, color) in enumerate(self.series):
            p.setPen(QColor(color))
            last = self.hist[i][-1] if self.hist[i] else 0.0
            p.drawText(x + i * 78, 21, f"{name} {last:+.2f}")

        vals = [v for hist in self.hist for v in hist if math.isfinite(v)]
        if self.fixed_range:
            ymin, ymax = self.fixed_range
        elif vals:
            ymin, ymax = min(vals), max(vals)
            if abs(ymax - ymin) < 1e-6:
                ymin -= 1.0
                ymax += 1.0
            pad = (ymax - ymin) * 0.12
            ymin, ymax = ymin - pad, ymax + pad
            # include zero when close
            if ymin > 0 and ymin < (ymax - ymin) * 2:
                ymin = 0
            if ymax < 0 and abs(ymax) < (ymax - ymin) * 2:
                ymax = 0
        else:
            ymin, ymax = -1.0, 1.0

        p.setPen(QPen(QColor("#243244"), 1))
        for j in range(5):
            yy = plot.top() + j * plot.height() / 4
            p.drawLine(QPointF(plot.left(), yy), QPointF(plot.right(), yy))
            val = ymax - j * (ymax - ymin) / 4
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(2, yy - 8, 42, 16), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, f"{val:.1f}")
            p.setPen(QPen(QColor("#243244"), 1))
        for j in range(5):
            xx = plot.left() + j * plot.width() / 4
            p.drawLine(QPointF(xx, plot.top()), QPointF(xx, plot.bottom()))

        zero_y = plot.bottom() - (0 - ymin) / (ymax - ymin) * plot.height()
        if plot.top() <= zero_y <= plot.bottom():
            p.setPen(QPen(QColor("#41536a"), 1.2))
            p.drawLine(QPointF(plot.left(), zero_y), QPointF(plot.right(), zero_y))

        nmax = max((len(x) for x in self.hist), default=0)
        if nmax < 2:
            return
        for hist, (_, color) in zip(self.hist, self.series):
            if len(hist) < 2:
                continue
            path = QPainterPath()
            for i, value in enumerate(hist):
                if not math.isfinite(value):
                    continue
                xx = plot.left() + i / max(1, nmax - 1) * plot.width()
                yy = plot.bottom() - (value - ymin) / (ymax - ymin) * plot.height()
                if path.elementCount() == 0:
                    path.moveTo(xx, yy)
                else:
                    path.lineTo(xx, yy)
            p.setPen(QPen(QColor(color), 1.6))
            p.drawPath(path)


class VectorBar(QWidget):
    """Compact XYZ magnitude bars used for accel/velocity diagnostics."""

    def __init__(self, title="Vector", parent=None):
        super().__init__(parent)
        self.title = title
        self.values = [0.0, 0.0, 0.0]
        self.setMinimumHeight(118)

    def set_values(self, x, y, z):
        self.values = [float(x), float(y), float(z)]
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#0f1721"))
        p.setPen(QColor(TEXT))
        f = QFont(self.font()); f.setBold(True)
        p.setFont(f)
        p.drawText(12, 22, self.title)

        colors = [QColor("#ff6b6b"), QColor("#69db7c"), QColor("#74c0fc")]
        labels = ["X", "Y", "Z"]
        scale = max(0.05, max(abs(v) for v in self.values))
        for i, (label, value, color) in enumerate(zip(labels, self.values, colors)):
            y = 43 + i * 23
            p.setPen(QColor(MUTED))
            p.drawText(12, y + 11, label)
            bar = QRectF(34, y, max(20, self.width() - 112), 13)
            p.setBrush(QColor("#192635")); p.setPen(Qt.PenStyle.NoPen)
            p.drawRoundedRect(bar, 5, 5)
            center = bar.center().x()
            width = min(bar.width() / 2, abs(value) / scale * bar.width() / 2)
            fill = QRectF(center if value >= 0 else center - width, y, width, 13)
            p.setBrush(color)
            p.drawRoundedRect(fill, 5, 5)
            p.setPen(QColor(TEXT))
            p.drawText(QRectF(self.width() - 72, y - 2, 64, 17),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       f"{value:+.3f}")
