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

## EEPROM emulasi

Page flash terakhir 1 KB dipakai sebagai EEPROM emulasi. Data dilindungi magic, version, length, dan CRC16. Yang disimpan saat ini adalah bias gyro awal dan parameter noise ESKF.

Peta flash:

- `0x08000000..0x080017FF`: bootloader 6 KB;
- `0x08001800..0x0800FBFF`: aplikasi 57 KB;
- `0x0800FC00..0x0800FFFF`: EEPROM emulasi 1 KB.

Area aplikasi berarti sekitar 89% dari total flash 64 KB tetap tersedia untuk firmware utama.

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

Saat aplikasi lama masih berjalan, reset/power-cycle board agar tool dapat menangkap jendela bootloader sekitar 800 ms.

## Build dan flash ST-LINK

```bash
pio run -e BOOTLOADER_STLINK
pio run -e BOOTLOADER_STLINK -t upload
pio run -e APP_STLINK
pio run -e APP_STLINK -t upload
```

## Catatan pengujian hardware saat implementasi

I2C telah terbukti mendeteksi satu perangkat di `0x68` dengan `WHO_AM_I=0x72`. Raw accel, gyro, suhu, serta ESKF sudah berjalan pada board nyata. Saat pengujian terakhir, CH340 terdeteksi Linux sebagai `/dev/ttyUSB0`, tetapi byte dari CH340 tidak masuk ke USART1 maupun USART2 MCU dan data MCU juga tidak diterima CH340. Artinya kabel TX/RX fisik ke adapter perlu diperiksa; firmware USART2 sendiri berjalan dan fungsi transmit melaporkan sukses.

## Upload firmware melalui USART

Aplikasi dan bootloader sama-sama memakai USART2 921600 baud. Tool `tools/flash_uart.py` otomatis:

1. mencoba mendeteksi bootloader;
2. bila aplikasi sedang aktif, mengirim command `ENTER_BOOTLOADER` (`0xF1`);
3. aplikasi menyimpan magic di backup register lalu software-reset;
4. bootloader melakukan `INFO -> ERASE -> WRITE -> VERIFY CRC -> GO`.

Dengan mekanisme ini update normal tidak memerlukan ST-LINK.

### RX USART yang robust

USART2 RX memakai interrupt `RXNE` dan ring buffer 64 byte. Tujuannya agar command servis, termasuk `ENTER_BOOTLOADER`, tidak hilang ketika CPU sedang melakukan transmit telemetry. Pendekatan ini mengikuti prinsip firmware sideboard asli yang juga memakai penerimaan USART secara asinkron.

Upload yang sudah diuji langsung:

```bash
pio run -e APP_USART -t upload
```

Pengujian dua upload berturut-turut berhasil: aplikasi masuk bootloader otomatis, erase, write, verifikasi CRC image penuh, lalu kembali menjalankan aplikasi tanpa reset manual dan tanpa ST-LINK.

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

Kalibrasi enam sisi bukan sekadar diputar terus. Tahan board stabil pada tiap orientasi sampai coverage sisi tersebut selesai. Firmware memakai rata-rata sampel stabil per sisi, menghitung offset/scale, memeriksa RMS dan error maksimum, lalu baru menyimpan ke EEPROM jika semua syarat lolos.

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
