#!/usr/bin/env python3
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IMU = ROOT / "tools" / "imu"
sys.path.insert(0, str(IMU))

from PySide6.QtWidgets import QApplication
from imu_gui import ImuDashboard


def sample():
    return {
        "command": 0xF2, "version": 5, "flags": 735,
        "seq": 123, "time_us": 456789,
        "accel_raw": (-400, -380, 8350), "temp_raw": 0, "gyro_raw": (12, 24, 36),
        "roll": -1.25, "pitch": 2.50, "yaw": 35.0,
        "vel": (0.1, -0.2, 0.05), "pos": (1.2, 0.3, -0.1),
        "linacc": (0.03, -0.04, 0.02), "cal": (2, 0x15, 0, 420),
        "aid_age_ms": 25, "aid_reject": 0,
        "att_std_deg": (0.2, 0.2, 0.4), "vel_std": (0.03, 0.04, 0.05),
        "pos_std": (0.1, 0.1, 0.2), "nav_status": 49, "health_resets": 0,
        "temp_c": 48.5, "imu_whoami": 0x72, "imu_class": 1,
        "observed_sample_hz": 200.5,
    }


class GuiSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_dashboard_renders_and_updates(self):
        w = ImuDashboard(auto_connect=False)
        w.show()
        d = sample()
        w.on_telemetry(d)
        w.on_stats({"rate": 50.0, "valid": 100, "lost": 0, "crc": 0, "other": 0, "reconnects": 0})
        self.app.processEvents()
        self.assertEqual(w.m_roll.value.text(), "-1.25")
        self.assertEqual(w.d_rate.value.text(), "50.0")
        self.assertEqual(w.cal_progress.value(), 420)
        w.on_config({"status":0,"sub":1,"output_map":0x0B})
        self.assertTrue(w.chk_ix.isChecked())
        self.assertTrue(w.chk_iy.isChecked())
        self.assertFalse(w.chk_iz.isChecked())
        self.assertTrue(w.chk_swap.isChecked())
        self.assertEqual(w.current_output_map(),0x0B)
        self.assertTrue(w.grab().size().width() > 1000)
        w.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)