#!/usr/bin/env python3
import os
import struct
import sys
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "tools", "imu")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import flash_uart
import read_imu

MAIN = os.path.join(ROOT, "Src", "main.c")
BOOT = os.path.join(ROOT, "Src", "bootloader.c")
CONFIG = os.path.join(ROOT, "Inc", "app_config.h")


def boot_info(version=4, start=0x08001800, end=0x0800F400, page=1024, chunk=128, valid=True):
    return (
        bytes((flash_uart.CMD_INFO, 0, version))
        + struct.pack(">IIHBB", start, end, page, chunk, int(valid))
    )


class Agent3IntegrationTests(unittest.TestCase):
    def test_source_specific_freshness_limits(self):
        cfg = open(CONFIG, encoding="utf-8").read()
        self.assertIn("#define AID_WHEEL_MAX_AGE_US                 50000UL", cfg)
        self.assertIn("#define AID_YAW_MAX_AGE_US                   50000UL", cfg)
        self.assertIn("#define AID_WORLD_VEL_MAX_AGE_US             50000UL", cfg)
        self.assertIn("#define AID_WORLD_POS_MAX_AGE_US             40000UL", cfg)
        main = open(MAIN, encoding="utf-8").read()
        self.assertIn("aid_source_max_age_us(req.type)", main)
        self.assertNotIn("age>AID_MAX_AGE_US", main)

    def test_stationary_startup_initializes_eskf_from_averaged_imu(self):
        main = open(MAIN, encoding="utf-8").read()
        self.assertIn("init_sample.accel_mps2[i]=startup.accel_avg[i]", main)
        self.assertIn("init_sample.gyro_rads[i]=startup.gyro_avg[i]", main)
        self.assertIn("init_sample.temperature_c=startup.temperature_avg", main)
        self.assertIn("init_filter(&eskf, &init_sample, &settings)", main)

    def test_reacquisition_guard_requires_three_candidates(self):
        cfg = open(CONFIG, encoding="utf-8").read()
        main = open(MAIN, encoding="utf-8").read()
        self.assertIn("#define AID_REACQUIRE_COUNT                      3U", cfg)
        self.assertIn("position_acquisition_offer", main)
        self.assertIn("yaw_acquisition_offer", main)
        self.assertIn("status=7U", main)

    def test_slip_observer_is_integrated_not_reimplemented(self):
        main = open(MAIN, encoding="utf-8").read()
        self.assertIn("slip_observer_update(&slip_observer", main)
        self.assertIn("slip_observer_wheel_sigma_scale", main)
        self.assertIn("slip_observer_nhc_sigma_scale", main)
        self.assertIn("slip_observer_reject_wheel", main)
        self.assertIn("expected_yaw_rate_valid=0U", main)
        self.assertIn("!telemetry_due && !board_uart_tx_busy()", main)

    def test_diagnostic_packet_decode(self):
        payload = struct.pack(
            ">BBhHHHBBBB7IH4IBII",
            read_imu.COMM_SIDEBOARD_DIAG, 1,
            -250, 725, 125, 640,
            2, 1, 4, 1,
            10, 20, 3, 4, 5, 6, 7,
            144,
            8, 9, 10, 11,
            1,
            12, 13,
        )
        self.assertEqual(len(payload), 69)
        d = read_imu.decode_diag(payload)
        self.assertAlmostEqual(d["wheel_innovation_mps"], -0.25)
        self.assertAlmostEqual(d["wheel_nis"], 7.25)
        self.assertAlmostEqual(d["effective_wheel_sigma_mps"], 0.125)
        self.assertEqual(d["slip_state"], 2)
        self.assertEqual(d["fifo_max_bytes"], 144)
        self.assertTrue(d["covariance_psd_ok"])
        self.assertEqual(d["filter_health_reset_count"], 13)

    def test_diagnostic_v2_appends_imu_gap_fields(self):
        prefix = struct.pack(
            ">BBhHHHBBBB7IH4IBII",
            read_imu.COMM_SIDEBOARD_DIAG, 2,
            -250, 725, 125, 640,
            2, 1, 4, 1,
            10, 20, 3, 4, 5, 6, 7,
            144,
            8, 9, 10, 11,
            1,
            12, 13,
        )
        payload = prefix + struct.pack(">IHH", 14, 125, 880)
        self.assertEqual(len(payload), 77)
        d = read_imu.decode_diag(payload)
        self.assertEqual(d["version"], 2)
        self.assertEqual(d["imu_gap_count"], 14)
        self.assertEqual(d["last_imu_gap_ms"], 125)
        self.assertEqual(d["max_imu_gap_ms"], 880)

    def test_legacy_v5_decode_unchanged(self):
        payload = bytearray(114)
        payload[0] = read_imu.COMM_SIDEBOARD_IMU
        payload[1] = 5
        data = read_imu.decode_imu(bytes(payload))
        self.assertIsNotNone(data)
        self.assertEqual(data["version"], 5)
        self.assertEqual(data["seq"], 0)

    def test_boot_info_is_read_only_and_begin_update_latches(self):
        src = open(BOOT, encoding="utf-8").read()
        info = src[src.index("if (p[0] == CMD_INFO)"):src.index("if (p[0] == CMD_BEGIN_UPDATE")]
        begin = src[src.index("if (p[0] == CMD_BEGIN_UPDATE"):src.index("if (p[0] == CMD_ERASE")]
        self.assertNotIn("boot_request_set()", info)
        self.assertIn("boot_request_set()", begin)

    def test_boot_verify_validates_vector_crc_before_commit(self):
        src = open(BOOT, encoding="utf-8").read()
        fn = src[src.index("static int verify_and_commit_app"):src.index("static int app_valid")]
        self.assertLess(fn.index("image_vectors_valid"), fn.index("write_manifest"))
        self.assertLess(fn.index("image_crc32_matches"), fn.index("write_manifest"))
        self.assertIn("if (current == hw) continue", src)
        self.assertIn("CMD_VERIFY && len == 9U", src)

    def test_v4_flash_flow_uses_begin_crc32_and_post_go_check(self):
        image = b"\x00" * 16
        seen = []

        def retry(_port, payload, expect_cmd, timeout, attempts=3):
            seen.append(bytes(payload))
            cmd = payload[0]
            if cmd == flash_uart.CMD_WRITE:
                return bytes((cmd, 0)) + payload[1:5]
            return bytes((cmd, 0))

        with mock.patch.object(flash_uart, "transact_retry", side_effect=retry),              mock.patch.object(flash_uart, "transact", return_value=bytes((flash_uart.CMD_GO, 0))),              mock.patch.object(flash_uart, "verify_application_boot", return_value=b"\xF0\x05"):
            flash_uart.flash_image(mock.Mock(), image, boot_info(4))

        self.assertEqual(seen[0], bytes((flash_uart.CMD_BEGIN_UPDATE,)))
        verify = next(p for p in seen if p[0] == flash_uart.CMD_VERIFY)
        self.assertEqual(len(verify), 9)
        self.assertEqual(struct.unpack(">I", verify[5:9])[0], flash_uart.crc32_image(image))

    def test_v3_host_flash_compatibility_uses_crc16(self):
        image = b"\x11" * 16
        seen = []

        def retry(_port, payload, expect_cmd, timeout, attempts=3):
            seen.append(bytes(payload))
            cmd = payload[0]
            if cmd == flash_uart.CMD_WRITE:
                return bytes((cmd, 0)) + payload[1:5]
            return bytes((cmd, 0))

        with mock.patch.object(flash_uart, "transact_retry", side_effect=retry),              mock.patch.object(flash_uart, "transact", return_value=bytes((flash_uart.CMD_GO, 0))),              mock.patch.object(flash_uart, "verify_application_boot", return_value=b"\xF0\x05"):
            flash_uart.flash_image(mock.Mock(), image, boot_info(3))

        self.assertFalse(any(p[0] == flash_uart.CMD_BEGIN_UPDATE for p in seen))
        verify = next(p for p in seen if p[0] == flash_uart.CMD_VERIFY)
        self.assertEqual(len(verify), 7)

    def test_missing_application_after_go_is_reported(self):
        port = mock.Mock()
        with mock.patch.object(flash_uart, "wait_for_application", return_value=None),              mock.patch.object(flash_uart, "catch_bootloader", side_effect=TimeoutError):
            with self.assertRaisesRegex(RuntimeError, "APP_BOOT_FAILED"):
                flash_uart.verify_application_boot(port, timeout=0.01)


if __name__ == "__main__":
    unittest.main(verbosity=2)
