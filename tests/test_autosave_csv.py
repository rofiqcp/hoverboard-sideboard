#!/usr/bin/env python3
import csv
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
TOOLS=ROOT/"tools"
IMU_TOOLS=TOOLS/"imu"
sys.path.insert(0,str(IMU_TOOLS))

import imu_log
from run_csv import run_logged
from tool_paths import LOG_DIR


class FakePort:
    def __init__(self):
        self.port="FAKE"
        self.is_open=True
    def reset_input_buffer(self): pass
    def close(self): self.is_open=False


def sample(seq):
    return {
        "version":5,"seq":seq,"time_us":seq*20000,"temp_c":25.0,
        "accel_raw":(0,0,8192),"gyro_raw":(0,0,0),
        "roll":0.0,"pitch":0.0,"yaw":0.0,
        "vel":(0.0,0.0,0.0),"pos":(0.0,0.0,0.0),"linacc":(0.0,0.0,0.0),
        "flags":0x5F,"nav_status":0x31,"cal":(0,0,0,0),
        "aid_age_ms":65535,"aid_reject":0,"imu_whoami":0x72,"imu_class":1,
        "observed_sample_hz":200.0,
    }


class AutosaveCsvTests(unittest.TestCase):
    def test_imu_data_is_closed_and_saved_on_ctrl_c(self):
        out=LOG_DIR/"autosave_interrupt_test.csv"
        if out.exists(): out.unlink()
        state={"n":0}
        def fake_read(_port):
            state["n"]+=1
            if state["n"]>25:
                raise KeyboardInterrupt
            return b"x",None
        def fake_decode(_payload):
            return sample(state["n"])
        argv=["imu_log.py","--duration","300","--output","autosave_interrupt_test.csv"]
        with mock.patch.object(sys,"argv",argv), \
             mock.patch.object(imu_log,"open_sideboard_port",return_value=FakePort()), \
             mock.patch.object(imu_log,"read_frame",side_effect=fake_read), \
             mock.patch.object(imu_log,"decode_imu",side_effect=fake_decode):
            rc=run_logged(imu_log.main,str(IMU_TOOLS/"imu_log.py"))
        self.assertEqual(rc,130)
        self.assertTrue(out.exists())
        with open(out,newline="") as f:
            rows=list(csv.DictReader(f))
        self.assertEqual(len(rows),25)
        self.assertEqual(int(rows[-1]["seq"]),25)


if __name__=="__main__":
    unittest.main(verbosity=2)
