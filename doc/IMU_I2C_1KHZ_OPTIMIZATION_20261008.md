# Optimasi STM32F103 MPU6xxx I²C → FIFO → ESKF → Python (2026-10-08)

## Lingkup dan status

Repo: `/media/sirobo/HDD/hoverboard-sideboard`; branch kerja: `highspeed-estimator-20261006`.

- **500 Hz** tetap default (baseline yang sudah ada). 
- **1.000 Hz** tersedia sebagai **profil bench/opt-in**, bukan otomatis aktif untuk kendaraan.
- Tidak ada STM32/USB-UART sideboard yang terhubung selama audit; hasil adalah **build + regression host**, belum hasil hardware live. Tidak dilakukan flash, commit, atau merge.
- Sensor ID yang pernah terbaca adalah `WHO_AM_I=0x72`, berarti kompatibel keluarga MPU65xx, **belum membuktikan model chip asli**. DLPF tetap CFG=2.

## Profil kompilasi

| Parameter | Baseline | 1 kHz candidate |
|---|---:|---:|
| MPU FIFO accelerometer + gyro | 500 Hz | 1000 Hz |
| I²C1 PB6/PB7 | 400 kHz | 400 kHz |
| FIFO record | 12 B | 12 B |
| FIFO software max batch | 8 | 16 |
| TIM2 hardware service | 100 Hz, ARR=9999 | 200 Hz, ARR=4999 |
| ESKF predict / preintegration take | 100 Hz | 200 Hz |
| Telemetry F0 ke Python | 50 Hz | 100 Hz |
| Temperature reads | ~10 Hz | ~10 Hz |
| Gravity correction opportunity (stationary) | 50 Hz | 50 Hz |
| Alternating velocity-ZUPT / zero-rate | 10 Hz each | 10 Hz each |
| Diagnostic F5 | opportunistic | opportunistic |

Profil 1kHZ pada `platformio.ini`:
`APP_STLINK_1KHZ` / `APP_USART_1KHZ` (build flag `-DIMU_PROFILE_1KHZ=1`).
Profil default tetap `APP_STLINK` / `APP_USART`.

## Perubahan yang benar-benar dilakukan

1. **Bug TIM2 diperbaiki**: register ARR tidak lagi hard-coded 100Hz dan mengikuti `IMU_LOOP_PERIOD_US-1`. Sebelumnya percobaan setting 200Hz di compile-time saja tidak mengubah timer real.
2. FIFO read `HAL_I2C_Mem_Read` tetap burst AG 12-byte/sample; buffer dan batas driver sama-sama mengikuti `IMU_FIFO_BATCH_CAPACITY`, sehingga batch 10 sampel bukan lagi false-overflow pada profil 1kHz.
3. Boot startup sample buffer juga mengikuti profil (menghindari inkonsistensi dengan service normal).
4. Frequency divisor temperatur, gravity, ZUPT dihitung dari service Hz sehingga laju koreksi tidak berubah diam-diam saat service digandakan.
5. Timeout I²C diturunkan dari 20ms menjadi **dua kelas transfer**: 5ms untuk pembacaan hingga 96 byte, 8ms untuk burst di atas 96 byte (maksimum 192 byte). Config writes 5ms. Operasi sehat seharusnya jauh lebih cepat. Jika bus macet, deadline miss tetap didiagnosis dan recovery I²C dipertahankan. Dua kelas juga menghemat flash daripada pembagian integer di hot-path.
6. `observed_sample_hz` kini menunjukkan rate hasil hitungan/EMA sesungguhnya, tidak lagi dipotong ke ±5% nominal. Timestamp per-sample tetap mempunyai fallback nominal jika rate input di luar rentang 80–120% agar propagation tidak memakai `dt` absurd.
7. Test host driver mencakup 1kHz batch 10 sampel, sequence dan jarak timestamp, overcapacity-resync, batas timeout I²C dan rate drop yang harus terdeteksi.
8. Build 1kHz dipisahkan dari default. `run_stm32_python_gate.sh` membangun dua profil, mengecek ukuran image terhadap **partition aplikasi 56.320 byte**, lalu menjalankan regression C dan Python.

## Angka throughput & batas keselamatan

- Pada 1kHz, 12.000 byte/s payload FIFO. Di I²C 400 kHz dengan 9 bit/byte minimum ±27% dari kapasitas wire (belum address/start/stop dan overhead).
- Normal service 200 Hz menerima ±5 sampel / 5 ms, yaitu 60 byte burst.
- 16 sampel = 192 byte FIFO, minimum wire-time 4,32 ms tanpa overhead. Ini hanya backlog tolerance, **bukan** kondisi normal yang dijamin menepati deadline 5ms.
- Chip FIFO 512 byte ≈ 42 record @12B; namun software sengaja reset saat backlog >16 untuk mencegah state memakai data terlalu tua. Resync juga harus memicu covariance gap inflation, **tidak boleh diabaikan**.
- Flash F103 app partition adalah 55 KiB = 56.320 byte. Image `firmware.bin` setelah optimasi berukuran sekitar **56.244 byte**, sehingga sisa ruang image sekitar **76 byte**. `Flash used` PlatformIO berbeda dari panjang BIN karena layout/alignment. Gate memeriksa panjang BIN terhadap partisi. **Jangan** menambah blok filter besar tanpa refactor ukuran program atau pindah MCU.
- Parameter UART 921600 baud masih jauh di atas kebutuhan telemetry 100 Hz, dan tidak perlu dinaikkan dulu.
- Frequency 1000 Hz adalah **ODR sampel**, bukan bandwidth 1000 Hz (DLPF ~92–98 Hz); 1000 Hz **tidak** menjamin akurasi posisi yang lebih tinggi.

## Pengujian software

```bash
cd /media/sirobo/HDD/hoverboard-sideboard
bash tools/imu/run_stm32_python_gate.sh

# opsi build satu profil
/home/sirobo/.platformio/penv/bin/pio run -e APP_STLINK_1KHZ
/home/sirobo/.platformio/penv/bin/pio run -e APP_USART_1KHZ

# keluaran setelah board disambungkan
python3 tools/imu/read_imu.py --mode nav --every 1
```

## HARDWARE GATE sebelum mengaktifkan 1kHz

1. **Pertama**, jalankan firmware baseline 500Hz dan `bash tools/imu/run_stm32_python_gate.sh --live 60` saat kendaraan diam. Simpan log baseline.
2. Verifikasi kabel I²C/pull-up PB6/PB7 dan identity sensor. Jangan gunakan asumsi MPU6500 asli hanya berdasarkan ID 0x72.
3. Flash build `APP_USART_1KHZ` / `APP_STLINK_1KHZ` **hanya pada bench yang aman**, dengan prosedur bootloader yang sudah tervalidasi untuk board tertentu.
4. Jalankan `bash tools/imu/run_stm32_python_gate.sh --live-1khz 60` stationary.
5. Acceptance target: observed IMU 950–1050Hz, NAV 95–105Hz, arrival P95 ≤15ms, sensor lag P95 ≤5ms; I²C error=0, FIFO overflow/resync=0, scheduler miss=0, IMU gap=0, UART errors=0, no clip, covariance PSD healthy.
6. Rekam 10 menit stationary dan bandingkan velocity RMS, position drift, yaw drift, sensor lag P95, jitter, suhu, dan status dengan baseline 500Hz.
7. Uji gerak perlahan dan dinamika kendaraan dengan sumber pembanding yang terukur; **jangan menganggap integrasi IMU-only bisa menghasilkan absolute XYZ stabil dalam jangka panjang**.
8. Jika satu saja error rate/timeout/deadline meningkat, tetap gunakan baseline 500Hz sampai akar masalah ditemukan. Jangan mengubah covariance/low-pass hanya untuk menutupi noise fisik.

## Optimasi tambahan hot path dan regression pemrosesan

- Bug temperature-cache raw nol diperbaiki. Raw suhu bernilai 0 valid; kini ada flag temperature_cache_valid terpisah agar I2C suhu tetap sekitar 10 Hz, bukan setiap service hingga 200 Hz.
- Memset per FIFO ImuSample dihapus karena semua field yang digunakan telah diisi; mengurangi CPU/memory traffic 500–1000 sampel/s.
- Kalibrasi ulang sampel terakhir tidak dilakukan dua kali setiap service; hanya bila kandidat kalibrasi mengubah settings.
- Test driver 500 dan 1000 Hz menguji suhu raw 0 tetap menjaga I2C budget. Semua PASS.
- Test matematis 1000 Hz raw -> 200 Hz ESKF memakai lintasan sintetis 20 detik: velocity_error=1.269066e-06 m/s dan position_error=7.39563933e-05 m. PASS.
- Angka sintetis tidak membuktikan presisi fisik atau deadline STM32. Hardware gate wajib sebelum profil 1 kHz menjadi default.

## Hal yang belum diklaim

- Tidak ada bukti hardware runtime 1kHz/200Hz pada F103 selama sesi ini.
- Tidak ada angka empiris CPU worst-case time; ukuran firmware/lulus test tidak mengukur deadline CPU.
- Tidak ada verifikasi angka sensor lag nyata P95 / FIFO packet loss / oscillator drift / I²C waveform.
- Tidak ada pengkabelan MPU INT/DATA_READY yang diverifikasi; jangan aktifkan IRQ tanpa pemeriksaan pin fisik.
- Presisi mutlak XYZ/VXYZ tetap tergantung bias, ZUPT yang valid, dan sensor aiding saat bergerak.

## File terkait

`Inc/app_config.h`, `Src/board.c`, `Src/imu_mpu6xxx.c`, `Src/main.c`, `platformio.ini`, `tests/test_agent1_imu_health.c`, `tests/test_agent3_integration.py`, `tools/imu/run_agent1_tests.sh`, `tools/imu/run_stm32_python_gate.sh`.
