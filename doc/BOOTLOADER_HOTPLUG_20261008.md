# Bootloader v4 & UART Hotplug — 2026-10-08

## Hasil perubahan pada branch aktif

- Host UART flasher mengunci adapter yang telah diverifikasi oleh telemetry aplikasi F0/F5, menggunakan USB VID/PID + serial number/lokasi USB sebelum F1. Setelah USB re-enumeration, hanya device yang identik secara fisik yang menjadi kandidat; identitas ambigu atau tanpa fingerprint stabil pada auto-mode ditolak. Port eksplisit dari operator tetap diperbolehkan.
- Firmware aplikasi diperiksa SEBELUM mengirim F1: panjang, Cortex-M vector stack pointer, Thumb reset handler, dan alamat handler di dalam image. Setelah bootloader INFO, host mencocokkan map persis 0x08001800..0x0800F400, page 1024, chunk <=128. Firmware salah atau bootloader perangkat lain tidak boleh memulai ERASE.
- Retry WRITE memeriksa ACK address secara tepat; ACK tertunda untuk chunk sebelumnya tidak diterima sebagai ACK saat ini. Parser buffer dibersihkan saat retry TimeoutError.
- Jika GO ACK hilang atau USB lepas saat GO, flasher mencari telemetry runtime valid terlebih dahulu melalui port yang terverifikasi. Status GO ambigu TIDAK memicu ERASE ulang. Hanya setelah runtime F0 valid flasher menyatakan berhasil.
- Saat WRITE/ERASE terputus, sesi gagal menutup port; retry sesi berikutnya mencari kembali adapter yang sama dan memulai ERASE+WRITE ulang (tanpa memakai data flash parsial sebagai firmware valid).
- Di bootloader STM32, cache `cached_app_valid` disetel false SEBELUM operasi erase sehingga erase parsial tidak dapat dilaporkan sebagai aplikasi valid.
- ROS 2 `serial_aiding_bridge.py` sticky ke identitas USB fisik yang diverifikasi pertama kali; USB yang re-enumerate dapat dipakai lagi, port adapter berbeda atau beberapa identitas ambigu tidak otomatis dipakai. Semua koneksi tetap butuh probe telemetri pasif CRC/version/size sebelum mengirim aiding.

## Pengujian software

- `tests/test_bootloader_hotplug.py` menguji relokasi ttyUSBn, mismatch/ambiguity, ACK F1/GO hilang, request ACK WRITE salah, GO ambiguous fail-safe, transfer WRITE yang perlu retry, serta firmware/map preflight.
- `tests/test_bootloader_virtual_uart.py` mengetes paket VESC CRC16 transport, BEGIN/ERASE/WRITE/VERIFY CRC32/GO secara utuh memakai virtual UART, ACK WRITE hilang, dan korupsi flash yang mencegah commit.
- `ros2/sideboard_p2_fusion/test/test_serial_protocol.py` mencakup identity-locked ROS bridge saat USB hotplug dan kondisi adapter salah/ambigu.
- Gate offline: `tools/run_full_robustness_gate.sh`. Tidak mengakses atau mem-flash hardware secara otomatis.

## Prosedur penggunaan

Pada board yang sudah menjalankan aplikasi dengan telemetry F0/F5 dan terhubung ke USB-UART:

```bash
cd /media/sirobo/HDD/hoverboard-sideboard
tools/run_full_robustness_gate.sh
# Pilih firmware sesuai board dan profil; auto hanya memulai dari aplikasi valid
python3 tools/imu/flash_uart.py .pio/build/APP_USART/firmware.bin --port auto --baud 921600 --retries 6 --overall-timeout 120
```

Jika proses flasher mati (bukan hanya kabel hotplug) ketika aplikasi sudah ter-ERASE, bootloader silent tidak dapat dibuktikan melalui probe F0/F5. Jalankan lagi memakai **port USB-UART yang operator telah verifikasi** (`--port /dev/serial/by-id/...` atau port eksplisit yang cocok); jangan melakukan pencarian aktif ke semua ttyUSB.

## Batasan kejujuran pengujian

- Semua hotplug sampai saat ini berbasis mocking/simulasi; sideboard dan ST-Link belum terhubung. Belum terverifikasi pemulihan hotplug fisik, power-cut flash, baud 921600 pada USB-UART spesifik, serta RTC/reset MCU.
- USB VID/PID/lokasi/serial dan CRC bukan autentikasi kriptografis. Penggunaan industri memerlukan wiring/udev stabil, eksklusivitas port, izin akses, dan commissioning dengan board nyata.
- Bootloader v4 berada di 0x08000000..0x080017FF (6 KiB); aplikasi 0x08001800..0x0800F3FF (55 KiB), area metadata dan journal tetap terpisah. Jangan flash bootloader tanpa recovery ST-Link dan backup settings.
- Default firmware 500 Hz; 1 kHz hanya untuk bench sampai deadline CPU, IMU FIFO, serial link, dan watchdog diuji pada hardware.

## Hotplug frame resynchronization hardening — follow-up

- Actual reproducible failure: a stream starting with a truncated `0x02, 0xC0` VESC header hides a following valid 118-byte telemetry or INFO frame until timeout. Before this fix `flash_uart.read_packet()` returned `None` while holding a complete valid response in its buffer.
- `tools/imu/flash_uart.py` now scans for complete CRC-valid frames behind incomplete/bogus prefixes, enforces maximum 192-byte payload, bounds buffer growth, and detects short serial writes before awaiting an ACK.
- Applied equivalent CRC-valid partial-prefix recovery to `tools/imu/read_imu.py`, passive `tools/imu/serial_common.py` identity probe, and ROS2 `ShortVescParser` in `serial_aiding_bridge.py`. USB UART probe still **never sends active commands** to an unidentified device.
- Regression: `tests/test_hotplug_parser_recovery.py` covers bootloader INFO, fragmented reads, IMU F0 decode, passive identity, rejecting foreign F0 layout, and short UART command TX. `test_serial_protocol.py` includes ROS chunked hotplug resync, a bounded-buffer noise soak and sideboard telemetry recovery.
- This work fixes a software parser fault. It does not prove a physical hotplug or power interruption passes; sideboard/ST-Link hardware was not detected during testing.
