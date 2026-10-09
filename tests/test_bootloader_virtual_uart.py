"""Virtual UART protocol test (not a substitute for real-board hotplug)."""
import struct
import sys
import zlib
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools"/"imu"))
import flash_uart as fw

class VirtualPort:
    def __init__(self, drop_write_ack=False, corrupt=False):
        self.port="virtual"
        self.rx=bytearray()
        self.flash=bytearray(b"\xff"*56320)
        self.manifest=False
        self.app=False
        self.erases=0
        self.writes=0
        self.drop_write_ack=drop_write_ack
        self.ack_dropped=False
        self.corrupt=corrupt
    @property
    def in_waiting(self):
        return len(self.rx)
    def flush(self): pass
    def reset_input_buffer(self): self.rx.clear()
    def write(self,packet):
        n=packet[1]
        payload=packet[2:2+n]
        assert packet[0]==2 and packet[-1]==3
        assert fw.crc16(payload)==int.from_bytes(packet[2+n:4+n],"big")
        cmd=payload[0]
        response=bytes((cmd,0))
        if cmd==fw.CMD_ERASE:
            self.flash[:]=b"\xff"*len(self.flash)
            self.manifest=False
            self.erases+=1
        elif cmd==fw.CMD_WRITE:
            address=int.from_bytes(payload[1:5],"big")
            off=address-fw.EXPECTED_APP_START
            part=payload[5:]
            assert 0<=off and off+len(part)<=len(self.flash)
            for k,b in enumerate(part):
                assert (self.flash[off+k] & b)==b
                self.flash[off+k]&=b
            if self.corrupt and off>0:
                self.flash[off]^=1
            self.writes+=1
            response+=payload[1:5]
        elif cmd==fw.CMD_VERIFY:
            length,expect=struct.unpack(">II",payload[1:9])
            self.manifest=(zlib.crc32(self.flash[:length])&0xffffffff)==expect
            response=bytes((cmd,0 if self.manifest else 1))
        elif cmd==fw.CMD_GO:
            self.app=self.manifest
            response=bytes((cmd,0 if self.app else 1))
        if self.drop_write_ack and cmd==fw.CMD_WRITE and not self.ack_dropped:
            self.ack_dropped=True
        else:
            self.rx.extend(fw.make_packet(response))
        if cmd==fw.CMD_GO and self.app:
            self.rx.extend(fw.make_packet(bytes((0xF0,6))+bytes(116)))
        return len(packet)
    def read(self,size):
        if not self.rx and self.app:
            self.rx.extend(fw.make_packet(bytes((0xF0,6))+bytes(116)))
        value=bytes(self.rx[:size])
        del self.rx[:size]
        return value

def image():
    return struct.pack("<II",0x20005000,0x08001809)+bytes(range(248))

def info():
    return bytes((fw.CMD_INFO,0,4))+struct.pack(">IIHBB",0x08001800,0x0800F400,1024,128,0)

def test_complete_flash_verify_and_jump_virtual_uart():
    port=VirtualPort()
    fw.flash_image(port,image(),info())
    assert port.erases==1 and port.writes==2
    assert port.manifest and port.app and bytes(port.flash[:256])==image()

def test_lost_write_ack_resends_identical_chunk():
    port=VirtualPort(drop_write_ack=True)
    fw.flash_image(port,image(),info())
    assert port.erases==1 and port.writes==3
    assert port.ack_dropped and port.app and bytes(port.flash[:256])==image()

def test_corrupted_flash_never_commits_manifest_or_jumps():
    port=VirtualPort(corrupt=True)
    with pytest.raises(RuntimeError,match="VERIFY gagal"):
        fw.flash_image(port,image(),info())
    assert port.erases==1 and not port.manifest and not port.app
