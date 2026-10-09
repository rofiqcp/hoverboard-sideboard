# P0 Robustness Hardening — 2026-10-08

## Ruang lingkup dan status
Bekerja pada branch `highspeed-estimator-20261006` dan mempertahankan perubahan lokal yang sudah ada. Tidak melakukan flash hardware, commit, atau menghapus file sumber. Gate offline: `tools/run_full_robustness_gate.sh`.

## Perbaikan di firmware STM32
- Perintah `CAL_CMD_STATIONARY_ON` sekarang memiliki **lease maksimum 2 detik** (`MASTER_STATIONARY_LEASE_US`), tidak berlaku selamanya bila Python/NUC terputus.
- `STATIONARY_ON` hanya di-ACK sukses jika IMU mendeteksi diam dan besar kecepatan ESKF lebih kecil dari 0.15 m/s. Di luar itu ACK dengan status 3 (rejected).
- Motion detector membatalkan master stationary dan startup-ZUPT. Izin gravitasi memerlukan `sample_is_still` yang dibuktikan sensor, bukan sekadar perintah master atau status kalibrasi.
- Untuk mendapatkan hold diam lebih dari 2 detik, controller harus mengirim ulang `STATIONARY_ON` hanya selama kendaraan *benar-benar diam*; setiap transisi akan bergerak harus mengirim `STATIONARY_OFF`.
- Status ini bukan bukti kendaraan diam ketika seluruh sensor eksternal gagal; IMU 6-axis tidak mampu membedakan diam dari gerak translasi konstan.
- Firmware build APP_STLINK, APP_USART, APP_STLINK_1KHZ, APP_USART_1KHZ, BOOTLOADER_STLINK: **PASS**. App partition 56,320 byte, image sekitar 56,296 (500Hz) dan 56,300 (1kHz); margin hanya 24/20 byte dan tidak aman menambahkan firmware lagi tanpa refaktor ukuran kode.

## Perbaikan komunikasi P2 / ROS2
- ROS serial bridge tidak lagi memilih sembarang `/dev/ttyUSB*`/`/dev/ttyACM*`; enumerasi dibatasi USB-UART VID yang didukung dan koneksi harus lulus probe **pasif** dengan frame telemetry F0/F5, CRC valid, tipe serta panjang yang sesuai. Probe tidak mengirim command.
- ACK wheel aiding modern (F4, 7-byte) harus sesuai request ID yang *pending*. ACK lama tanpa ID atau ID salah tidak meningkatkan count valid.
- Pending request kedaluwarsa menurut `ack_timeout_s=0.25` dan tercatat sebagai `ack_timeout_count`. Diagnostik menjadi WARN ketika ACK belum ada, stale, ditolak, atau timeout baru saja terjadi. Status OK tidak lagi dihasilkan hanya karena port terbuka.
- P2 melacak expiry external slip data `external_slip_timeout_s=0.50` pada domain waktu ROS; toleransi kecil untuk perbedaan waktu header dan arrival.
- Jika LIO stale, probabilitas slip tidak otomatis kembali sehat; wheel covariance sekurangnya `stale_reference_sigma_floor_mps=0.50`. Status diagnostik mengindikasikan LIO atau wheel stale.
- Quaternion LIO nol/nonfinite ditolak. Quaternion valid dinormalisasi saat ekstraksi yaw. Covariance LiDAR nonfinite/negatif diberi uncertainty 1e6, bukan otomatis diperlakukan presisi.

## Perbaikan monitoring dan logging
- `imu_log.py` mengosongkan cache F5 setiap serial reconnect. Health diagnostik yang hilang atau lebih lama 1 detik dicatat kosong/UNKNOWN, ditambahkan `diag_fresh` dan `diag_age_ms`.
- `analyze_nav_log.py --stationary` kini **FAIL CLOSED** untuk kolom F5 yang hilang, diagnosa stale, kesehatan yang tidak dapat dipastikan, output NaN/Inf, counter gangguan, atau hilangnya umur sample. Semua nilai kesehatan harus dibuktikan oleh data, bukan default sehat.
- `validate_nav_stream.py` mewajibkan F5 setidaknya dua frame, rate diagnostik minimal 3 Hz, gap maksimal 750 ms, serta F5 terakhir lebih baru dari 1 detik.

## Regression hasil aktual
```text
tools/run_full_robustness_gate.sh
Firmware builds: APP_STLINK, APP_USART, BOOTLOADER_STLINK, APP_STLINK_1KHZ, APP_USART_1KHZ PASS
C ESKF/IMU/slip/preintegration suites PASS
Python tests 44 PASS
P2 ROS2 + GUI tests 27 PASS
git diff --check PASS
HARDWARE VALIDATION NOT RUN: sideboard serial/ST-Link not detected
```

## Batas yang masih wajib dilakukan sebelum produksi
1. Flash firmware ke board yang identitasnya terverifikasi, mulai dari 500 Hz; pastikan bootloader/EEPROM dan fitur emergency tidak terganggu.
2. Test 30–60 menit pada 500 Hz dan 1 kHz untuk deadline scheduler, CPU/stack headroom, FIFO overflow/resync, I2C NACK/SCL hold, sensor time lag P95.
3. Pada kendaraan diam dan pengujian aman, uji ON lease 2 detik, pemutusan komunikasi, dan gerak setelah stationary.
4. Cabut/pasang USB-UART saat ROS P2 berjalan; verifikasi tidak pernah mengirim ke ESC/Android/IMU USB lain dan ACK timeout tertangkap.
5. Lakukan loss-of-LIO + wheel-slip-in-motion pada rig aman, periksa wheel covariance dan odometry; uji power-cut journal EEPROM.
6. Kalibrasi imu noise, temperatur, six-face dan Allan; validasi XYZ/VXYZ terhadap LiDAR/RTK/UWB ground truth. Simulasi preintegration bukan akurasi fisik.

## Prosedur
```bash
cd /media/sirobo/HDD/hoverboard-sideboard
tools/run_full_robustness_gate.sh
# Hanya setelah sideboard terhubung dan flashed dan board benar-benar diam:
tools/imu/run_stm32_python_gate.sh --live 60
# Versi 1kHz adalah bench candidate, bukan default kendaraan:
tools/imu/run_stm32_python_gate.sh --live-1khz 60
```

Catatan: Status ini software-ready, bukan hardware-qualified. Konfigurasi 1 kHz tetap terpisah dan tidak diaktifkan sebagai default.
