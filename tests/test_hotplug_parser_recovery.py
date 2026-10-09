"""Malformed partial USB-UART frames must not hide subsequent valid frames."""
import sys
from pathlib import Path
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"tools"/"imu"))
import flash_uart
import read_imu
import serial_common

class BufferedSerial:
    def __init__(self, data, *, chunk_size=None):
        self.data=bytearray(data)
        self.chunk_size=chunk_size
        self._vesc_rx_buffer=bytearray()
        self.is_open=True

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, n):
        if self.chunk_size:
            n=min(n,self.chunk_size)
        n=min(n,len(self.data))
        result=bytes(self.data[:n])
        del self.data[:n]
        return result

    def reset_input_buffer(self):
        pass


def test_bootloader_recovers_valid_info_behind_truncated_192byte_candidate():
    payload=bytes((flash_uart.CMD_INFO,0,4))+bytes(12)
    port=BufferedSerial(b"\\x02\\xc0"+flash_uart.make_packet(payload))
    assert flash_uart.read_packet(port,.1)==payload


def test_bootloader_corrupted_partial_prefix_with_chunked_reads_recovers():
    payload=bytes((flash_uart.CMD_INFO,0,4))+bytes(12)
    port=BufferedSerial(b"\\x02\\xc0"+"noise".encode()+
                        flash_uart.make_packet(payload),chunk_size=3)
    assert flash_uart.read_packet(port,.2)==payload


def test_imu_stream_resynchronizes_after_partial_192byte_prefix():
    payload=bytes((0xF0,6))+bytes(116)
    port=BufferedSerial(b"\\x02\\xc0"+serial_common._vesc_packet(payload))
    out,err=read_imu.read_frame(port)
    assert out==payload and err is None


def test_passive_identity_probe_recovers_crc_valid_payload_after_false_prefix():
    payload=bytes((0xF0,6))+bytes(116)
    port=BufferedSerial(b"\\x02\\xc0"+serial_common._vesc_packet(payload))
    assert serial_common._probe_sideboard_identity(port,seconds=.10)


def test_passive_identity_never_accepts_crc_valid_foreign_frame():
    payload=bytes((0xF0,6))+bytes(100) # wrong sideboard layout
    port=BufferedSerial(b"\\x02\\xc0"+serial_common._vesc_packet(payload))
    with mock.patch.object(serial_common.time,"sleep",return_value=None):
        assert not serial_common._probe_sideboard_identity(port,seconds=.02)


def test_short_uart_command_write_is_rejected_without_waiting_for_ack():
    class PartialTx:
        def write(self, frame):return len(frame)-1
        def flush(self):raise AssertionError("flush should not be reached")
    import serial
    try:
        flash_uart.transact(PartialTx(),bytes((flash_uart.CMD_INFO,)),
                            flash_uart.CMD_INFO,timeout=.1)
        raise AssertionError("short TX accepted")
    except serial.SerialTimeoutException:
        pass
