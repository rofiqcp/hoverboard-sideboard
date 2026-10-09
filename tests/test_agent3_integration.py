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
import aiding_imu

MAIN = os.path.join(ROOT, "Src", "main.c")
IMU_DRIVER = os.path.join(ROOT, "Src", "imu_mpu6xxx.c")
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
        self.assertIn("#define AID_WHEEL_MAX_AGE_US                 15000UL", cfg)
        self.assertIn("#define AID_YAW_MAX_AGE_US                   50000UL", cfg)
        self.assertIn("#define AID_WORLD_VEL_MAX_AGE_US             50000UL", cfg)
        self.assertIn("#define AID_WORLD_POS_MAX_AGE_US             40000UL", cfg)
        main = open(MAIN, encoding="utf-8").read()
        self.assertIn("aid_source_max_age_us(req.type)", main)
        self.assertNotIn("age>AID_MAX_AGE_US", main)


    def test_highspeed_imu_profile_is_explicit_and_verified(self):
        cfg = open(CONFIG, encoding="utf-8").read()
        driver = open(IMU_DRIVER, encoding="utf-8").read()
        self.assertIn("#define MPU6XXX_HIGHSPEED_PROFILE       1U", cfg)
        self.assertIn("#define MPU6XXX_SAMPLE_HZ           500UL", cfg)
        self.assertIn("#define MPU6XXX_ACCEL_LSB_PER_G      4096.0f", cfg)
        self.assertIn("#define MPU6XXX_GYRO_LSB_PER_DPS       32.8f", cfg)
        self.assertIn("#define MPU6XXX_DLPF_CFG                2U", cfg)
        self.assertIn("MPU6XXX_GYRO_CONFIG_VALUE", driver)
        self.assertIn("MPU6XXX_ACCEL_CONFIG_VALUE", driver)
        self.assertIn("nominal_hz*0.80f", driver)

    def test_1khz_profile_timer_tracks_service_frequency(self):
        cfg = open(CONFIG, encoding="utf-8").read()
        board = open(os.path.join(ROOT, "Src", "board.c"), encoding="utf-8").read()
        main = open(MAIN, encoding="utf-8").read()
        driver = open(IMU_DRIVER, encoding="utf-8").read()
        self.assertIn("#define IMU_PROFILE_1KHZ             0U", cfg)
        self.assertIn("#define MPU6XXX_SAMPLE_HZ          1000UL", cfg)
        self.assertIn("#define IMU_LOOP_RATE_HZ            200UL", cfg)
        self.assertIn("#define TELEMETRY_RATE_HZ           100UL", cfg)
        self.assertIn("TIM2->ARR = IMU_LOOP_PERIOD_US - 1U", board)
        self.assertIn("ImuSample fifo_samples[IMU_FIFO_BATCH_CAPACITY]", main)
        self.assertIn("ImuSample samples[IMU_FIFO_BATCH_CAPACITY]", main)
        self.assertIn("max_samples > IMU_FIFO_BATCH_CAPACITY", driver)
        self.assertIn("IMU_TEMP_READ_DIVIDER", driver)
        self.assertIn("IMU_GRAVITY_DIVIDER", main)
        self.assertIn("IMU_ZUPT_DIVIDER", main)

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

    def test_short_delayed_wheel_history_compensation_is_present(self):
        cfg = open(CONFIG, encoding="utf-8").read()
        main = open(MAIN, encoding="utf-8").read()
        self.assertIn("#define AID_WHEEL_HISTORY_SAMPLES                 3U", cfg)
        self.assertIn("#define AID_WHEEL_COMP_MIN_AGE_US              2000UL", cfg)
        self.assertIn("wheel_history_lookup(&wheel_history,now_us,age,&old_vx)", main)
        self.assertIn("target[0]+=current_body[0]-old_vx", main)
        self.assertIn("wheel_history_reset(&wheel_history)", main)
        self.assertIn("last_sensor_sample_us=0U", main)

        # z_now = z_old + (h_now-h_old) preserves the old measurement residual.
        z_old, h_old, h_now = 9.4, 9.1, 9.8
        z_now = z_old + (h_now - h_old)
        self.assertAlmostEqual(z_now - h_now, z_old - h_old)

    def test_slip_observer_is_integrated_not_reimplemented(self):
        main = open(MAIN, encoding="utf-8").read()
        self.assertIn("slip_observer_update(&slip_observer", main)
        self.assertIn("slip_observer_wheel_sigma_scale", main)
        self.assertIn("slip_observer_nhc_sigma_scale", main)
        self.assertIn("slip_observer_reject_wheel", main)
        self.assertIn("AID_FLAG_EXPECTED_YAW_RATE", main)
        self.assertIn("eskf_nav_preview_body_x_nis", main)
        self.assertIn("!telemetry_due && !board_uart_tx_busy()", main)

    def test_wheel_v2_optional_expected_yaw_rate_is_backward_compatible(self):
        base = dict(command="wheel", values=[3.25], sigma=0.05, age_ms=0.0,
                    board_timestamp=None, frame="local", nhc=True)
        old = mock.Mock(**base, expected_yaw_rate=None)
        ext = mock.Mock(**base, expected_yaw_rate=0.75)
        old_payload, _, _ = aiding_imu.build_payload(old, 1, 0x1234)
        ext_payload, _, _ = aiding_imu.build_payload(ext, 1, 0x1234)
        self.assertEqual(len(old_payload), 17)
        self.assertEqual(len(ext_payload), 21)
        self.assertEqual(old_payload[16], 1)
        self.assertEqual(ext_payload[16], 3)
        self.assertEqual(struct.unpack(">i", ext_payload[17:21])[0], 750)

    def test_fifo_timestamp_drives_preintegration_dt(self):
        main = open(MAIN, encoding="utf-8").read()
        driver = open(IMU_DRIVER, encoding="utf-8").read()
        self.assertIn("last_fifo_sample_time_us + sample_period_us", driver)
        self.assertIn("first_sample_time_us + (uint32_t)n * sample_period_us", driver)
        self.assertIn("raw.sample_time_us-last_sensor_sample_us", main)
        self.assertIn("sample_dt=(float)dt_us*1.0e-6f", main)

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
        struct.pack_into(">i", payload, 38, 1234)
        data = read_imu.decode_imu(bytes(payload))
        self.assertIsNotNone(data)
        self.assertEqual(data["version"], 5)
        self.assertEqual(data["seq"], 0)
        self.assertAlmostEqual(data["vel"][0], 1.234, places=3)

    def test_v6_decode_exposes_sensor_epoch_and_highspeed_raw_scale(self):
        payload = bytearray(118)
        payload[0] = read_imu.COMM_SIDEBOARD_IMU
        payload[1] = 6
        struct.pack_into(">I", payload, 8, 1_000_000)   # snapshot board time
        struct.pack_into(">h", payload, 12, 4096)       # +1 g at +/-8 g
        struct.pack_into(">h", payload, 20, 328)        # +10 dps at +/-1000 dps
        struct.pack_into(">i", payload, 38, 12345)       # vx = 1.2345 m/s in v6
        struct.pack_into(">i", payload, 50, 23456)       # x = 2.3456 m in v6
        struct.pack_into(">i", payload, 62, 34567)       # ax_world = 3.4567 m/s2
        payload[104:114] = struct.pack(">iBBI", 25000, 0x72, 2 | (2 << 2) | (2 << 4), 500000)
        payload[114:118] = struct.pack(">I", 998000)
        data = read_imu.decode_imu(bytes(payload))
        self.assertIsNotNone(data)
        self.assertEqual(data["version"], 6)
        self.assertEqual(data["sample_time_us"], 998000)
        self.assertEqual(data["sensor_lag_us"], 2000)
        self.assertEqual(data["accel_lsb_per_g"], 4096.0)
        self.assertEqual(data["accel_fs_sel"], 2)
        self.assertEqual(data["gyro_fs_sel"], 2)
        self.assertEqual(data["gyro_lsb_per_dps"], 32.8)
        self.assertAlmostEqual(data["accel_raw"][0] / data["accel_lsb_per_g"], 1.0)
        self.assertAlmostEqual(data["gyro_raw"][0] / data["gyro_lsb_per_dps"], 10.0)
        self.assertAlmostEqual(data["vel"][0], 1.2345, places=4)
        self.assertAlmostEqual(data["pos"][0], 2.3456, places=4)
        self.assertAlmostEqual(data["linacc"][0], 3.4567, places=4)

    def test_v6_sensor_lag_handles_u32_wrap(self):
        payload = bytearray(118)
        payload[0] = read_imu.COMM_SIDEBOARD_IMU
        payload[1] = 6
        struct.pack_into(">I", payload, 8, 0x000003E8)  # 1000 us after wrap
        meta = 2 | (2 << 2) | (2 << 4)
        payload[104:114] = struct.pack(">iBBI", 25000, 0x72, meta, 500000)
        payload[114:118] = struct.pack(">I", 0xFFFFFF00)
        data = read_imu.decode_imu(bytes(payload))
        self.assertIsNotNone(data)
        self.assertEqual(data["sensor_lag_us"], 1256)

    def test_startup_zupt_is_one_shot_after_motion(self):
        main = open(MAIN, encoding="utf-8").read()
        cfg = open(CONFIG, encoding="utf-8").read()
        self.assertNotIn("auto_zupt_rearm_enabled", main)
        self.assertNotIn("auto_rearm_still_count", main)
        self.assertNotIn("AUTO_ZUPT_REARM_STILL_SAMPLES", cfg)
        self.assertIn("if (!sample_is_still) { master_stationary=0U; startup_zupt=0U; }", main)
        self.assertIn("int allow_zupt = sample_is_still", main)
        self.assertIn("MASTER_STATIONARY_LEASE_US", main)

    def test_telemetry_keeps_one_canonical_frame(self):
        main = open(MAIN, encoding="utf-8").read()
        self.assertNotIn("apply_output_mapping(", main)
        self.assertNotIn("output_map_vec3(", main)
        self.assertIn("if(map==0U)", main)
        self.assertIn("mounting harus lewat sensor_to_body_q", main)

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
        image = struct.pack("<II", 0x20005000, 0x08001809) + b"\x00" * 8
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
        image = struct.pack("<II", 0x20005000, 0x08001809) + b"\x11" * 8
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
