# High-Speed Estimator Upgrade — 2026-10-06

## Tujuan
Meningkatkan robustness deteksi kecepatan, posisi dead-reckoning jangka pendek, IMU timing, dan deteksi wheel slip pada AGV berkecepatan tinggi tanpa mengubah memory map STM32F103C8.

## Perubahan yang sudah diterapkan

### 1. Firmware kembali muat ke STM32F103C8
Build sebelumnya overflow 3380 byte pada region aplikasi 55 KiB.
Build flags sekarang memakai LTO dan size optimizations tanpa mengubah alamat bootloader/aplikasi/settings.
Memory map tetap:
- bootloader: 0x08000000..0x080017FF
- application: 0x08001800..0x0800F3FF
- settings A: 0x0800F400
- manifest: 0x0800F800
- settings B: 0x0800FC00

Final checked APP image: 55388 bytes; app region 56320 bytes; headroom sekitar 932 bytes.

### 2. Current-measurement wheel NIS
Slip observer tidak lagi memakai NIS measurement sebelumnya.
API baru:
`eskf_nav_preview_body_x_nis()`
menghitung innovation/NIS wheel yang sedang datang tanpa mengubah nominal state atau covariance.
Urutan runtime:
wheel measurement -> preview innovation/NIS -> slip observer -> adaptive sigma/reject -> ESKF fusion.

### 3. High-speed slip response
Slip observer dituning agar deadband tidak tumbuh terlalu besar terhadap kecepatan:
- EWMA tau: 0.35 s -> 0.20 s
- moving residual base: 0.28 -> 0.24 m/s
- speed deadband gain: 0.08 -> 0.02
- full forward evidence: 1.20 -> 0.80 m/s
- suspect hold: 0.35 -> 0.15 s
- slip hold: 0.65 -> 0.30 s
- severe hold: 0.90 -> 0.45 s

Regression tambahan:
- 3% mismatch pada 10 m/s tetap NORMAL.
- 10% mismatch persisten pada 10 m/s terdeteksi sebagai SLIP/SEVERE.

### 4. Expected yaw-rate untuk Ackermann slip detection
Wheel aiding v2 sekarang dapat membawa optional expected yaw-rate.
Flag:
`AID_FLAG_EXPECTED_YAW_RATE = 1 << 1`

Paket lama 17-byte tetap diterima.
Paket extended 21-byte menambahkan signed int32 expected yaw-rate dalam 0.001 rad/s.

Host tool:
`python3 tools/imu/aiding_imu.py wheel 3.0 --sigma 0.05 --nhc --expected-yaw-rate 0.45`

Untuk kendaraan Ackermann, sender sebaiknya menghitung:
`yaw_rate_expected = velocity_mps / wheelbase_m * tan(steering_angle_rad)`

Gunakan steering angle aktual dari encoder, bukan command steering, jika tersedia.

### 5. IMU FIFO timestamp dipakai untuk preintegration
Sebelumnya `sample_time_us` hanya dibuat tetapi integration dt memakai satu nilai konstan `1/observed_sample_hz`.
Sekarang FIFO membentuk timeline sample monoton dan main loop menghitung dt dari timestamp antar sample.
Fallback ke nominal observed sample dt hanya bila interval timestamp tidak masuk range 0.5..20 ms.

Ini memisahkan physical IMU sample interval dari jitter scheduler/service MCU.

### 6. Wheel aiding freshness diperketat
`AID_WHEEL_MAX_AGE_US` diubah 50 ms -> 30 ms.
Local MCU belum memiliki history rewind/replay, sehingga wheel measurement lama tidak boleh terlalu jauh dari state saat fusion.

### 7. Runtime covariance health vs host full PSD
Full 15x15 Cholesky PSD check tetap tersedia dan tetap diuji pada host regression.
Firmware runtime memakai `eskf_nav_is_healthy()` setiap prediction cycle:
- nominal finite/range
- covariance diagonal checks
- full Cauchy bound scan
- lima PSD check blok 3x3

Ini mengurangi flash/CPU F103 tetapi mempertahankan fault detection runtime.

## Regression gate yang lulus
- host ESKF/preintegration math suite
- Agent 1 IMU runtime-health suite
- Agent 2 ESKF/slip suite
- Python integration/serial/autosave: 27 passed
- APP_STLINK build: PASS
- APP_USART build: PASS
- BOOTLOADER_STLINK build: PASS

Build final:
- APP RAM: 2068 / 20480 bytes
- APP flash image: 55388 bytes
- bootloader flash: 2624 bytes

## Yang belum boleh diklaim sebelum test hardware
Perubahan ini sudah compile dan regression-tested tetapi belum otomatis membuktikan performa kendaraan nyata.
Wajib lakukan:
1. stationary 10–30 menit;
2. straight run beberapa level speed;
3. acceleration/braking keras;
4. belok radius besar/kecil;
5. wheel-spin/skid terkontrol pada area aman;
6. log wheel innovation/NIS, slip score/state, accel/gyro clipping, FIFO resync, scheduler miss;
7. bandingkan odometry terhadap Livox/GNSS/ground truth;
8. ukur latency wheel measurement aktual sebelum menurunkan freshness di bawah 30 ms.

## IMU dynamic range
Production profile masih 200 Hz, +/-4 g, +/-500 deg/s, DLPF cfg 3.
Jangan menaikkan range/filter secara buta. Gunakan telemetry:
- accel_clip_count
- gyro_clip_count
- fifo_resync_count
- scheduler_miss_count

Jika clipping nyata muncul, benchmark profile 500 Hz dengan +/-8 g dan +/-1000 deg/s serta bandwidth yang lebih tinggi.

## Referensi upgrade tahap berikut
- UMich-CURLY/slip_detection_DOB — IMU + wheel encoder, slip/DOB concept
  https://github.com/UMich-CURLY/slip_detection_DOB
- XihangYU630/inekf_wheeled — slip-robust InEKF for wheeled robots
  https://github.com/XihangYU630/inekf_wheeled
- UMich-CURLY/husky_inekf — IMU + body/wheel velocity InEKF
  https://github.com/UMich-CURLY/husky_inekf
- i2Nav-WHU/KF-GINS — ODO/NHC + lever-arm reference
  https://github.com/i2Nav-WHU/KF-GINS
- ArduPilot EKF3 — delayed sensor timestamps and wheel odometry architecture
  https://github.com/ArduPilot/ardupilot
- HKU-MARS FAST-LIO — LiDAR-inertial high-dynamic external odometry anchor
  https://github.com/hku-mars/FAST_LIO

## Arsitektur target
STM32 sideboard:
IMU FIFO -> calibrated timestamped preintegration -> local ESKF -> wheel/NHC aiding -> slip-adaptive covariance.

NUC:
Livox Mid360 LIO + wheel + steering + sideboard IMU/odom + GNSS/RTK when available.

Untuk posisi jangka panjang, LiDAR/GNSS/vision/UWB harus menjadi external anchor. IMU + satu wheel forward velocity tidak cukup untuk membuat posisi absolut bounded selamanya.
