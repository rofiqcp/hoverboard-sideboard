#!/usr/bin/env python3
import contextlib
import io
import os
import sys
import unittest
from unittest import mock

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS=os.path.join(ROOT,"tools","imu")
if TOOLS not in sys.path: sys.path.insert(0,TOOLS)

import serial
import serial_common
import read_imu
import flash_uart
import calibrate_imu
import configure_imu

class FakePort:
    def __init__(self,name="FAKE"):
        self.port=name
        self.is_open=True
    def reset_input_buffer(self): pass
    def write(self,data): return len(data)
    def flush(self): pass
    def open(self): self.is_open=True
    def close(self): self.is_open=False

def imu_data(seq,time_us):
    return {
        "seq":seq,"time_us":time_us,"accel_raw":(0,0,8192),"gyro_raw":(0,0,0),
        "temp_raw":0,"temp_c":25.0,"roll":0.0,"pitch":0.0,"yaw":0.0,
        "linacc":(0.0,0.0,0.0),
        "vel":(0.0,0.0,0.0),"pos":(0.0,0.0,0.0),"vel_std":(0.0,0.0,0.0),
        "aid_age_ms":0,"aid_reject":0,"imu_whoami":0x72,"imu_class":1,
        "observed_sample_hz":200.0,"flags":3,"nav_status":1,"health_resets":0,
    }

class BufferPort:
    def __init__(self,data):
        self.data=bytearray(data)
        self.is_open=True
    @property
    def in_waiting(self): return len(self.data)
    def read(self,n):
        if not self.data: return b""
        n=min(n,len(self.data))
        out=bytes(self.data[:n]); del self.data[:n]
        return out

class RobustnessTests(unittest.TestCase):
    def test_read_frame_resync_after_corruption(self):
        payload=bytes((read_imu.COMM_SIDEBOARD_IMU,))+bytes(78)
        c=read_imu.crc16(payload)
        frame=bytes((2,len(payload)))+payload+bytes((c>>8,c&255,3))
        port=BufferPort(b"noise\x02\x00bad"+frame)
        got,err=read_imu.read_frame(port)
        self.assertEqual(got,payload)
        self.assertIsNone(err)

    def test_flash_parser_resync_after_corruption(self):
        payload=b"\xF8\x00\x03"
        c=flash_uart.crc16(payload)
        frame=bytes((2,len(payload)))+payload+bytes((c>>8,c&255,3))
        port=BufferPort(b"junk\x02\x01\x99\x00\x00"+frame)
        self.assertEqual(flash_uart.read_packet(port,.2),payload)

    def test_reader_recovers_midstream_disconnect(self):
        p1,p2=FakePort("COM18"),FakePort("COM18")
        opens=iter((p1,p2))
        state={"calls":0,"seq":0}
        def fake_open(*a,**k): return next(opens)
        def fake_read(port):
            state["calls"]+=1
            if port is p1 and state["calls"]==2:
                raise serial.SerialException("simulated unplug")
            return b"x",None
        def fake_decode(_):
            state["seq"]+=1
            return imu_data(state["seq"],state["seq"]*20000)
        out=io.StringIO()
        argv=["read_imu.py","--port","COM18","--count","3","--every","999","--mode","raw"]
        with mock.patch.object(sys,"argv",argv), \
             mock.patch.object(read_imu,"open_sideboard_port",side_effect=fake_open), \
             mock.patch.object(read_imu,"read_frame",side_effect=fake_read), \
             mock.patch.object(read_imu,"decode_imu",side_effect=fake_decode), \
             mock.patch.object(read_imu.time,"sleep",return_value=None), \
             contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            read_imu.main()
        text=out.getvalue()
        self.assertIn("reconnect=1",text)
        self.assertIn("valid=3",text)

    def test_open_port_retries_until_available(self):
        fake=FakePort("COM18")
        fake.dtr=False; fake.rts=False; fake.is_open=False
        attempts={"n":0}
        def find(_):
            attempts["n"]+=1
            if attempts["n"]<3: raise FileNotFoundError("missing")
            return "COM18"
        with mock.patch.object(serial_common,"find_sideboard_port",side_effect=find), \
             mock.patch.object(serial_common.serial,"Serial",return_value=fake), \
             mock.patch.object(serial_common.time,"sleep",return_value=None):
            got=serial_common.open_sideboard_port("COM18",attempts=5)
        self.assertIs(got,fake)
        self.assertEqual(attempts["n"],3)


    def test_ready_mode_formats_final_data(self):
        port=FakePort("COM18")
        state={"seq":0}
        def fake_read(_):
            state["seq"]+=1
            return b"x",None
        def fake_decode(_):
            return imu_data(state["seq"],state["seq"]*20000)
        out=io.StringIO()
        argv=["read_imu.py","--port","COM18","--count","1","--every","1","--mode","ready"]
        with mock.patch.object(sys,"argv",argv), \
             mock.patch.object(read_imu,"open_sideboard_port",return_value=port), \
             mock.patch.object(read_imu,"read_frame",side_effect=fake_read), \
             mock.patch.object(read_imu,"decode_imu",side_effect=fake_decode), \
             contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            read_imu.main()
        self.assertIn("READY",out.getvalue())
        self.assertIn("LinAcc=",out.getvalue())



    def test_config_output_map_decode(self):
        payload=bytearray(129)
        payload[0]=configure_imu.COMM
        payload[1]=configure_imu.SUB["get"]
        payload[2]=0
        payload[127]=0
        payload[128]=0x0B
        d=configure_imu.decode(bytes(payload),configure_imu.SUB["get"])
        self.assertIsNotNone(d)
        self.assertEqual(d["status"],0)
        self.assertEqual(d["output_map"],0x0B)

    def test_cal_request_accepts_telemetry_as_confirmation(self):
        port=FakePort("COM18")
        link=calibrate_imu.CalLink("COM18")
        link.port=port; link.last_name="COM18"
        data=imu_data(1,20000)
        data["cal"]=(calibrate_imu.STATE_STILL,0,0,120)
        with mock.patch.object(calibrate_imu,"read_frame",return_value=(b"x",None)), \
             mock.patch.object(calibrate_imu,"decode_imu",return_value=data):
            st=link.one_request(calibrate_imu.CMDS["still"],.1,
                                accepted_states={calibrate_imu.STATE_STILL})
        self.assertEqual(st["state"],calibrate_imu.STATE_STILL)
        self.assertEqual(st["source"],"telemetry")
        self.assertIs(link.port,port)


    def test_cal_silent_bootloader_auto_go(self):
        port=FakePort("COM18")
        link=calibrate_imu.CalLink("COM18")
        link.port=port; link.last_name="COM18"
        link.last_rx=0.0; link.last_boot_probe=0.0
        info=b"dummy"
        with mock.patch.object(calibrate_imu,"catch_bootloader",return_value=info), \
             mock.patch.object(calibrate_imu,"parse_info",return_value=(3,0x08002800,0x08010000,1024,128,True)), \
             mock.patch.object(calibrate_imu,"transact",return_value=bytes((calibrate_imu.CMD_GO,0))), \
             mock.patch.object(calibrate_imu.time,"sleep",return_value=None):
            self.assertTrue(link._recover_from_bootloader_if_needed())
        self.assertIsNone(link.port)
        self.assertFalse(port.is_open)

    def test_noncal_command_requires_real_ack(self):
        port=FakePort("COM18")
        link=calibrate_imu.CalLink("COM18")
        link.port=port; link.last_name="COM18"
        data=imu_data(1,20000)
        data["cal"]=(calibrate_imu.STATE_DONE,0x3f,0,1000)
        with mock.patch.object(calibrate_imu,"read_frame",return_value=(b"x",None)), \
             mock.patch.object(calibrate_imu,"parse_cal_status",return_value=None), \
             mock.patch.object(calibrate_imu,"decode_imu",return_value=data):
            with self.assertRaises(TimeoutError):
                link.one_request(calibrate_imu.CMDS["zero-nav"],.02)
        self.assertIs(link.port,port)

    def test_cal_timeout_does_not_close_healthy_com(self):
        port=FakePort("COM18")
        link=calibrate_imu.CalLink("COM18")
        link.port=port; link.last_name="COM18"
        with mock.patch.object(calibrate_imu,"read_frame",return_value=(None,"timeout")):
            with self.assertRaises(TimeoutError):
                link.one_request(calibrate_imu.CMDS["status"],.02)
        self.assertIs(link.port,port)
        self.assertTrue(port.is_open)

    def test_cal_stdin_thread_is_lazy(self):
        self.assertIsNone(calibrate_imu._STDIN_THREAD)

    def test_calibration_console_queue(self):
        calibrate_imu._INPUT_QUEUE.put("q")
        self.assertEqual(calibrate_imu._poll_command(),"q")

if __name__=="__main__":
    unittest.main(verbosity=2)
