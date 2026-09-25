# IMU workflow

Run all commands from the repository root.

Windows:
```
py tools/imu.py
```

Linux:
```
python3 tools/imu.py
```

## Recommended order

```
1 RAW
  ->
2 STILL calibration
  ->
3 accelerometer 6-side calibration
  ->
4 optional precision / thermal calibration
  ->
5 READY data
```

The main menu intentionally hides engineering/maintenance functions under **Advanced**.

## 1. Check RAW sensor data

Menu: **1. Baca IMU RAW**

Direct command:
```
<python> tools/imu.py raw --port auto
```

Check that accelerometer, gyro, temperature, and sample rate are alive and stable.
The serial summary should show no CRC errors and no sequence loss.

## 2. STILL calibration

Menu: **2. Kalibrasi STILL (gyro bias)**

Keep the board completely stationary until calibration completes.

Direct command:
```
<python> tools/imu.py still --port auto
```

Expected result: calibration DONE, status 0, error 0.

## 3. Accelerometer 6-side calibration

Menu: **3. Kalibrasi accelerometer 6 sisi**

Hold these six faces stationary, one at a time:

```
+X  -X  +Y  -Y  +Z  -Z
```

Direct command:
```
<python> tools/imu.py six-side --port auto
```

Wait for each face to be accepted. Complete coverage is 0x3F.

## 4. Optional advanced calibration

Menu: **4. Kalibrasi lanjutan (presisi / thermal)**

### 18-24 pose accelerometer fit

Use the submenu to record each stationary pose. Files are saved as:

```
tools/logs/accel_pose_01.csv
tools/logs/accel_pose_02.csv
...
```

Then fit:
```
<python> tools/imu.py ellipsoid "accel_pose_*.csv"
```

Apply only if the quality gate passes:
```
<python> tools/imu.py ellipsoid "accel_pose_*.csv" --apply
```

### Thermal calibration

Record at least three stable-temperature plateaus, with the board kept in the same physical orientation:

```
tools/logs/thermal_low.csv
tools/logs/thermal_mid.csv
tools/logs/thermal_high.csv
```

Fit:
```
<python> tools/imu.py thermal "thermal_*.csv"
```

Apply only if the fit is valid:
```
<python> tools/imu.py thermal "thermal_*.csv" --apply
```

After applying thermal calibration, stabilize the board at normal operating temperature and run **STILL calibration once again**.

## 5. Read final READY data

Menu: **5. Baca DATA FINAL siap pakai**

Direct command:
```
<python> tools/imu.py ready --port auto
```

READY shows only processed data intended for application use:
- roll / pitch / yaw
- linear acceleration
- velocity
- position
- velocity uncertainty
- temperature
- calibration / health / navigation flags

## 6. Record complete structured CSV

Menu: **6. Rekam data lengkap ke CSV**

Direct command:
```
<python> tools/imu.py log --port auto --duration 60 --output final_test.csv
```

Structured sensor files are always stored under:
```
tools/logs/
```

## Automatic logs and safe close

Every run of `tools/imu.py` and every directly executed `tools/imu/*.py` creates an automatic console/run CSV under:

```
tools/logs/run/
```

Logs are flushed continuously. Normal exit, Ctrl+C, Ctrl+Break, and SIGTERM close files cleanly.

## Individual modules

Engineering users may still run individual modules directly from:

```
tools/imu/
```

Examples:
```
<python> tools/imu/read_imu.py --mode raw --port auto
<python> tools/imu/calibrate_imu.py still --port auto
<python> tools/imu/calibrate_imu.py rotate-start --port auto
<python> tools/imu/read_imu.py --mode ready --port auto
```

Replace `<python>` with `py` on Windows or `python3` on Linux.
