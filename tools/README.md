# Sideboard IMU Tools

Gunakan satu entry point utama dari root repository:

Windows:
```
py tools/imu.py
```

Linux:
```
python3 tools/imu.py
```

Menu utama sengaja dibuat sederhana:

1. Baca IMU RAW
2. Kalibrasi STILL (gyro bias)
3. Kalibrasi accelerometer 6 sisi
4. Kalibrasi lanjutan (18-24 pose / thermal)
5. Baca DATA FINAL siap pakai
6. Rekam data lengkap ke CSV
7. Status kalibrasi
8. Advanced / maintenance

Alur normal untuk user:
```
RAW -> STILL -> 6-SISI -> (opsional presisi/thermal) -> READY
```


## GUI Qt6 interaktif

Jalankan dari root repository:

Windows:
```
py tools/imu.py gui
```

Linux:
```
python3 tools/imu.py gui
```

Atau jalankan `tools/imu.py` lalu pilih **G. GUI Dashboard Qt6**.

Fitur utama:
- satu ownership serial untuk seluruh GUI; tidak ada beberapa proses yang berebut COM
- auto reconnect/hotplug dan retry command tanpa menutup link hanya karena ACK terlambat
- animasi 3D body berdasarkan roll/pitch/yaw
- drag model 3D untuk orbit, mouse-wheel untuk zoom, double-click untuk reset camera
- artificial horizon + heading
- live plots RPY, velocity, raw acceleration, dan raw gyro
- calibration STILL dan 6-side dengan progress/coverage visual
- ZERO NAV, ZUPT, Stationary ON/OFF, cancel calibration
- health/flags/aiding diagnostics
- telemetry CSV otomatis per session ke `tools/logs/gui_session_*.csv`

Dependency GUI:
```
python -m pip install -r tools/imu/requirements-gui.txt
```


## Axis mapping dan Reset All

Setting axis disimpan di EEPROM dan hanya diterapkan pada output/telemetry; frame internal ESKF tidak diubah.

Bit mask:
- bit 0 = Invert X
- bit 1 = Invert Y
- bit 2 = Invert Z
- bit 3 = Swap Roll/Pitch

Terminal interaktif:
```
py tools/imu.py
```
Pilih **9. Axis / Output Mapping + Reset All**.

Command langsung:
```
py tools/imu.py config get --port auto
py tools/imu.py config output-map 1 --port auto
py tools/imu.py config output-map 8 --port auto
py tools/imu.py config output-map 0 --port auto
py tools/imu.py config reset-all --port auto
```

Mask dapat digabung, misalnya `9 = Invert X + Swap Roll/Pitch`.

Di GUI Qt6, kontrol yang sama ada di halaman **Calibration → Axis / Output Mapping**:
- Invert X
- Invert Y
- Invert Z
- Swap Roll / Pitch
- Apply Mapping
- Refresh
- Reset Mapping
- RESET ALL

**RESET ALL bersifat destruktif**: calibration STILL, 6-side, thermal, mount, noise tuning, lever arm, dan mapping kembali ke default. Setelah Reset All, lakukan calibration STILL dan 6-side kembali.

## RAW vs READY

RAW menampilkan data sensor sebelum dipakai estimator:
- accelerometer raw
- gyro raw
- temperature
- sample rate

READY menampilkan data hasil kalibrasi + filter/ESKF:
- roll, pitch, yaw
- linear acceleration
- velocity
- position
- velocity uncertainty
- temperature
- calibration/health/nav flags

## File internal

Semua implementasi ada di:
```
tools/imu/
```

Masing-masing tetap bisa dijalankan satu per satu, misalnya:
```
py tools/imu/read_imu.py --mode raw --port auto
py tools/imu/calibrate_imu.py still --port auto
py tools/imu/calibrate_imu.py rotate-start --port auto
py tools/imu/read_imu.py --mode ready --port auto
```

Di Linux, ganti `py` dengan `python3`.

## Komunikasi dinamis / hotplug

- Timeout ACK tidak dianggap USB disconnect; COM tetap dipertahankan.
- Kalibrasi membaca state/progress terutama dari telemetry 50 Hz, sehingga ACK yang terlambat tidak mereset sesi.
- USB/serial error nyata memicu reconnect otomatis.
- Jika STM32 reset ke bootloader dan image aplikasi valid, tool dapat mendeteksi bootloader saat link silent dan mengirim GO kembali ke aplikasi.
- STILL dan 6-sisi dapat menunggu/resume setelah hotplug/reset; Ctrl+C menutup port dengan bersih.
- RAW/READY membedakan `COM terbuka tetapi STM32 silent` dari `COM hilang`.

## CSV

- data sensor terstruktur: `tools/logs/*.csv`
- log otomatis setiap program: `tools/logs/run/*.csv`

Semua output memakai path relatif. Normal exit, Ctrl+C, Ctrl+Break, dan SIGTERM melakukan flush/close otomatis.

Workflow detail: `tools/imu/IMU_WORKFLOW.md`.
