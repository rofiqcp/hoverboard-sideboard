# Firmware Sideboard IMU Sederhana

Firmware aktif di repository ini sengaja dibuat hanya untuk satu tugas utama:

`IMU -> kalibrasi bias gyro -> ESKF attitude -> serial USART2`

Kode sideboard lama seperti DMP InvenSense, sensor optik, iBUS, LED demo, dan komunikasi hoverboard lama tidak ikut dikompilasi pada environment final.

## Hardware yang terdeteksi

- MCU: STM32F103C8T6.
- IMU: perangkat kompatibel keluarga MPU6xxx pada I2C alamat 7-bit `0x68`.
- Board yang diuji mengembalikan `WHO_AM_I = 0x72`, sehingga bukan MPU6050 original ID `0x68`, tetapi register accel/gyro dasarnya kompatibel.
- I2C1: PB6 = SCL, PB7 = SDA, 100 kHz.
- Serial utama: USART2 PA2 = TX, PA3 = RX, 921600 baud.
- ST-LINK: dipakai untuk build/upload dan diagnosis langsung.

## Pengolahan attitude

Filter memakai ESKF attitude 6-state:

- nominal state: quaternion body ke world;
- error state: `dtheta[3]` dan `gyro_bias[3]`;
- prediksi memakai gyro dikurangi bias;
- accelerometer dipakai sebagai pengamatan arah gravitasi;
- magnitude gate dan NIS gate menolak percepatan yang tidak layak difusi;
- koreksi terhadap arah gravitasi diproyeksikan agar tidak membuat koreksi yaw palsu;
- covariance memakai Joseph form agar lebih stabil pada float32 STM32F103.

Tanpa magnetometer atau referensi heading eksternal, yaw **bukan absolut**. Roll dan pitch dikoreksi gravitasi, sedangkan yaw tetap dapat drift perlahan.

## Data serial

Framing mengikuti paket pendek VESC:

`0x02 | panjang | payload | CRC16_hi | CRC16_lo | 0x03`

CRC16 memakai polynomial `0x1021` dan initial value `0`, sama dengan firmware VESC.

Payload command `0xF0` versi 1 berisi:

- status flags;
- sequence;
- timestamp mikrodetik;
- accel raw X/Y/Z;
- temperature raw;
- gyro raw X/Y/Z;
- roll, pitch, yaw dalam millidegree.

Baca dari Linux:

```bash
python3 tools/read_imu.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baud 921600
```

## Settings journal A/B

Kalibrasi/config tidak lagi disimpan pada satu page. Firmware memakai journal A/B power-loss-safe dengan generation counter, CRC payload, dan commit marker yang ditulis paling akhir. Record lama tetap dipertahankan sampai record baru selesai diverifikasi dan committed.

Peta flash:

- `0x08000000..0x080017FF`: bootloader 6 KiB;
- `0x08001800..0x0800F3FF`: aplikasi maksimum 55 KiB;
- `0x0800F400..0x0800F7FF`: settings journal A 1 KiB;
- `0x0800F800..0x0800FBFF`: manifest/commit marker aplikasi 1 KiB;
- `0x0800FC00..0x0800FFFF`: settings journal B / legacy IMU4 1 KiB.

Linker aplikasi berhenti tepat sebelum journal-A sehingga firmware tidak dapat menimpa settings, manifest, atau journal-B. Bootloader erase aplikasi juga berhenti di alamat yang sama.

## Bootloader UART

Bootloader memakai USART2 921600 baud dan framing VESC yang sama. Command yang tersedia:

- `INFO`;
- `ERASE`;
- `WRITE`;
- `VERIFY` CRC16 image lengkap;
- `GO`.

Tool host:

```bash
python3 tools/flash_uart.py .pio/build/APP_STLINK/firmware.bin
```

Bootloader normal memberi jendela sekitar 1,5 detik setelah reset. Update melalui aplikasi tidak perlu reset manual: command `0xF1` menyimpan recovery latch di backup register lalu software-reset, sehingga bootloader tetap aktif sampai `GO` berhasil.

## Build dan flash ST-LINK

```bash
pio run -e BOOTLOADER_STLINK
pio run -e BOOTLOADER_STLINK -t upload
pio run -e APP_STLINK
pio run -e APP_STLINK -t upload
```

## Catatan pengujian hardware saat implementasi

I2C telah terbukti mendeteksi perangkat `0x68` dengan `WHO_AM_I=0x72`. Raw accel/gyro/suhu, FIFO ~200 Hz, ESKF, native VESC IMU, serta upload aplikasi melalui USART2 921600 sudah diuji langsung pada board. Tool host memprioritaskan `/dev/serial/by-id/...` lalu fallback ke `/dev/ttyUSB*`/`ttyACM*`; board yang diuji saat ini muncul sebagai Prolific PL2303.

Pada sesi Tahap 2 pernah terjadi USB hub host error Linux `-71` yang memutus seluruh downstream hub (CH340 ikut hilang sementara ST-LINK kemudian enumerate kembali). Itu adalah fault link/hub host, bukan ESKF/UART firmware. Tool host sekarang melakukan retry-open untuk transien serial, tetapi perangkat yang benar-benar hilang dari USB tetap perlu dipulihkan di level hub/kabel/power.

## Upload firmware melalui USART

Aplikasi dan bootloader sama-sama memakai USART2 921600 baud. Tool `tools/flash_uart.py` otomatis:

1. mencoba mendeteksi bootloader;
2. bila aplikasi sedang aktif, mengirim command `ENTER_BOOTLOADER` (`0xF1`);
3. aplikasi menyimpan magic di backup register lalu software-reset;
4. bootloader melakukan `INFO -> ERASE -> WRITE -> VERIFY CRC -> GO`.

Dengan mekanisme ini update normal tidak memerlukan ST-LINK.

### RX USART yang robust

USART2 RX memakai interrupt `RXNE` dan ring buffer 128 byte. Overflow dihitung, byte tertua dibuang, dan parser mereset frame parsial saat overflow/inter-byte timeout. Parser juga dapat resinkron setelah garbage atau frame terpotong. ACK/control memiliki prioritas terhadap telemetry async.

Upload yang sudah diuji langsung:

```bash
pio run -e APP_USART -t upload
```

Pengujian dua upload berturut-turut berhasil: aplikasi masuk bootloader otomatis, erase, write, verifikasi CRC image penuh, lalu kembali menjalankan aplikasi tanpa reset manual dan tanpa ST-LINK.

### Bootloader v3: recovery/power-loss safety

Bootloader v3 memakai manifest commit marker terpisah. `ERASE` meng-invalidasi manifest **sebelum** menghapus aplikasi. `VERIFY` memeriksa CRC16 seluruh image, kemudian baru menulis manifest sebagai commit terakhir. Karena itu reset/listrik mati di tengah `WRITE` tidak pernah membuat image parsial dianggap valid. Setelah reset, bootloader tetap menunggu host sampai image lolos CRC dan `GO` diterima.

Aplikasi dan bootloader menggunakan independent watchdog. Jika main-loop aplikasi benar-benar deadlock, watchdog mereset MCU; bootloader memberi recovery window lebih panjang setelah watchdog reset. Fatal `board_panic()` juga diarahkan ke bootloader, bukan hard-lock.

Tool `flash_uart.py` mempunyai sliding frame resynchronizer, reconnect lintas USB re-enumeration, retry sesi penuh, port exclusive, dan deadline global. Jika ACK `WRITE`/`VERIFY` hilang atau USB putus, tool tidak mengasumsikan sukses: sesi berikutnya acquire bootloader lagi dan mengulang `ERASE -> WRITE -> VERIFY`.

Fault injection yang sudah diuji langsung: frame parsial, 400-byte garbage burst, flood 100 command, 5 siklus `F1 -> INFO -> GO`, serta reset setelah hanya 1280 byte image ditulis. Pada test reset-mid-update, `INFO` setelah reset tetap melaporkan `app_valid=False`, kemudian full recovery 42868 byte berhasil dengan CRC `0xB251` dan `GO` tanpa reset manual.

## Pemakaian paling sederhana

Port serial dipilih otomatis: firmware/tool memprioritaskan `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`, lalu fallback ke `/dev/ttyUSB0`. Baud aplikasi dan bootloader sama-sama **921600**.

```bash
# Baca data terus-menerus tanpa menulis nama port
python3 tools/read_imu.py

# Console live + command kalibrasi interaktif
python3 tools/calibrate_imu.py

# Kalibrasi gyro saat board benar-benar diam, tunggu sampai DONE dan auto-save EEPROM
python3 tools/calibrate_imu.py still

# Wizard kalibrasi accelerometer enam sisi (+X,-X,+Y,-Y,+Z,-Z)
python3 tools/calibrate_imu.py rotate-start

# Status kalibrasi
python3 tools/calibrate_imu.py status

# Upload aplikasi tanpa ST-LINK
pio run -e APP_USART -t upload
```

Kalibrasi enam sisi bukan sekadar diputar terus. Tahan board stabil pada tiap orientasi sampai coverage sisi tersebut selesai. Firmware memakai rata-rata sampel stabil per sisi, menghitung offset + transform matrix 3x3, memeriksa RMS dan error maksimum, lalu baru menyimpan ke EEPROM jika semua syarat lolos.

Saat master benar-benar mengetahui AGV berhenti, kirim `stationary-on`; saat mulai bergerak kirim `stationary-off`. Ini mengizinkan ZUPT/zero-rate membantu velocity, position, dan gyro-bias. Jangan mempertahankan stationary saat kendaraan bergerak konstan.

## Catatan validitas estimator

ESKF memakai 15 error-state: attitude error, velocity XYZ, position XYZ, gyro-bias XYZ, dan accel-bias XYZ. Nominal state memakai quaternion. Gravity correction tidak boleh mengobservasi yaw. Karena MPU6xxx ini hanya 6-axis, yaw tidak absolut dan velocity/position saat bergerak lama tetap dapat drift tanpa wheel odometry/GNSS/heading eksternal. Raw accel/gyro selalu tetap dikirim pada packet extended sehingga data asli tidak hilang.

`COMM_GET_IMU_DATA=65` mengikuti format native VESC: roll/pitch/yaw radian, accel dalam g, gyro deg/s, magnetometer nol, dan quaternion. Response hanya diberikan setelah estimator siap; selama startup tidak dikirim state nol palsu.

## Tahap 1 ESKF float (2026-09-17)

Jalur aktif sengaja tetap floating-point seperti pola ArduPilot/PX4; eksperimen fixed-point tidak dipakai.

Alur runtime final Tahap 1:

```text
MPU6xxx FIFO 200 Hz
  -> kalibrasi statik
  -> delta-angle + delta-velocity preintegrator + coning correction
  -> ESKF nominal 100 Hz (TIM2 hardware scheduler)
  -> covariance 50 Hz
  -> gravity correction 50 Hz
  -> ZUPT / zero-rate saat stationary
  -> VESC native IMU + extended telemetry 50 Hz
```

Catatan penting:
- `dt` FIFO mengikuti observed sample-rate sensor, bukan diasumsikan persis 200 Hz.
- Startup calibration memakai sampel FIFO berbeda selama sekitar 2 detik.
- Gagal kalibrasi tidak mengubah settings aktif/EEPROM (transactional commit).
- Startup ZUPT dilepas hanya setelah motion persisten; master `stationary-off` melepasnya deterministik sebelum AGV bergerak.
- Yaw tetap relative tanpa heading aiding eksternal.
- Position/velocity IMU-only tetap dead-reckoning; Tahap 2 perlu wheel odometry / external aiding untuk bounded navigation saat kendaraan bergerak.

## Tahap 2 navigation aiding + calibration IMU4 / telemetry v5 (2026-09-17)

EEPROM sekarang schema `IMU4` dan otomatis migrasi dari `IMU3`. Field baru meliputi full accelerometer transform 3x3, quaternion `sensor_to_body`, gyro/accel thermal slope, dan IMU lever-arm terhadap body origin. Loader memvalidasi CRC, finite/range seluruh parameter kritis, determinant matrix, dan quaternion.

Alur kalibrasi accelerometer enam sisi sekarang memakai model `a_corrected = T * (a_raw - offset)`. Enam centroid +X/-X/+Y/-Y/+Z/-Z membentuk matrix sensor 3x3, kemudian firmware menghitung inverse matrix dan memeriksa RMS/max residual sebelum transactional commit. Runtime hanya 9 multiply per sampel.

Command konfigurasi privat tetap memakai framing+CRC VESC:

```bash
python3 tools/configure_imu.py get
python3 tools/configure_imu.py mount-rpy ROLL PITCH YAW
python3 tools/configure_imu.py lever X Y Z
python3 tools/configure_imu.py thermal GX GY GZ AX AY AZ
python3 tools/configure_imu.py clear-thermal
python3 tools/configure_imu.py reset-mount
```

External aiding menggunakan command `0xF4`, timestamp MCU opsional, sigma measurement, absolute input/range guards, innovation/NIS gate, dan source reacquisition reset. Contoh:

```bash
python3 tools/aiding_imu.py wheel 1.0 --sigma 0.05 --nhc
python3 tools/aiding_imu.py world-vel 1.0 0.0 0.0 --sigma 0.10
python3 tools/aiding_imu.py world-pos 2.0 1.0 0.0 --sigma 0.25
python3 tools/aiding_imu.py yaw 90 --sigma 2
```

Wheel aiding mengobservasi body-forward velocity; NHC menahan body lateral/vertical velocity. Lever arm memakai koreksi `omega x r`. World velocity, world position, dan yaw memakai source reset ketika pertama acquire/ketika source timeout, lalu innovation fusion normal sesudah lock. Ini mencegah estimator menolak heading/velocity awal yang jauh dari state IMU-only.

Protocol extended **v5** mempertahankan prefix v4 dan menambahkan temperatur hasil konversi firmware, `WHO_AM_I`, device class, serta observed FIFO sample-rate. Navigation validity/covariance status dan health-reset counter tetap dikirim. Native `COMM_GET_IMU_DATA=65` tetap kompatibel dan tidak berubah.

Safety ESKF Tahap 2 mencakup finite/range guards, absolute innovation guards, NIS gates, bounded correction step, realistic gyro/accel residual-bias limits, covariance/state health check, serta automatic estimator recovery bila state menjadi non-finite/tidak sehat. Yaw aiding tidak diizinkan menyeret gyro-bias residual hingga batas.

### Kalibrasi presisi, thermal, dan identifikasi noise

Logger resmi untuk data calibration/noise:

```bash
python3 tools/imu_log.py --duration 120 --output records/still.csv
```

Untuk thermal, ambil beberapa plateau dengan **orientasi board tetap sama** dan temperatur berbeda, lalu fit:

```bash
python3 tools/thermal_calibration.py records/t25.csv records/t35.csv records/t45.csv
```

Untuk noise/Q, rekam stationary panjang (disarankan >=10 menit untuk bias walk) lalu:

```bash
python3 tools/allan_analysis.py records/still_long.csv --curve-out records/allan.csv
```

Untuk accelerometer presisi, gunakan >=12 orientasi diam berbeda dan host ellipsoid fit; runtime firmware tetap memakai matrix 3x3 yang ringan:

```bash
python3 tools/accel_ellipsoid_calibration.py records/pose01.csv records/pose02.csv records/pose03.csv ...
```

Six-face firmware tetap tersedia sebagai field-calibration sederhana. Tool host hanya memberi rekomendasi/aplikasi jika quality gate lolos; log terlalu pendek ditolak.

### External aiding frame/timing

Command `0xF4` sekarang memiliki kontrak eksplisit: `body`, `local Z-up`, atau `ENU`; timing dapat `now`, `board timestamp`, atau **measurement age**. Untuk ROS/NUC gunakan `--age-ms` kecuali clock MCU-host sudah disinkronkan. World velocity/position ENU ditolak sampai yaw ENU berhasil meng-align local world. Full ESKF re-init menghapus alignment itu sehingga source ENU wajib reacquire yaw.

Contoh:

```bash
python3 tools/aiding_imu.py yaw 90 --frame enu --age-ms 20 --sigma 2
python3 tools/aiding_imu.py world-vel 1 0 0 --frame enu --age-ms 20 --sigma 0.10
```

Regression host dapat dijalankan tanpa board:

```bash
./tools/run_host_math_tests.sh
```

Test tersebut memakai source C firmware yang sama untuk stationary propagation, yaw/velocity/position source reset, extreme measurement rejection, full 3x3 six-face calibration, dan transactional failure.
