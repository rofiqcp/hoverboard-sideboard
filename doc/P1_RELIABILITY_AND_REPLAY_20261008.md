# P1 Reliability, Time Ordering, Calibration Persistence, and Fault Replay — 2026-10-08

## Scope
This extends `doc/P0_ROBUSTNESS_HARDENING_20261008.md` without overwriting the dirty worktree. Focus on STM32 MPU6xxx, Python serial, and ROS2 P2 wheel/LIO estimator. **No physical board was attached or flashed.** The active 500 Hz firmware remains the default; 1 kHz is bench-only.

## Changes implemented and tested

1. **Persistent P2 calibration is fail-closed on invalid YAML floats.** `P2FusionCore.restore()` ignores NaN/Inf, nonnumeric values, and invalid individual fields, rather than letting NaN enter RLS covariance, steering zero, or wheelbase. Valid settings remain bounded.
2. **Monotonic wheel, LiDAR, and slip inputs.** Duplicate/out-of-order wheel odometry raises a controlled `ValueError` before any state update. Out-of-order LIO reference updates and LIO pose publications are ignored, so the differentiation baseline cannot jump backward. Older external slip values do not renew its validity lease.
3. **Power-loss durability on calibration save.** After an atomic `os.replace()`, Linux/POSIX explicitly `fsync()`s the parent directory, in addition to `fsync()` on the written file. This cannot protect against arbitrary storage hardware faults but closes the rename durability gap.
4. **Strengthened USB sideboard identity.** Beyond the VESC frame CRC and command ID, CLI and ROS bridge now validate the exact supported (version, payload length) combinations: F0 v5/114, v6/118 for P2; CLI additionally supports historical F0 versions 2/79, 3/101, 4/104; diagnostic F5 v1/69, v2/77. No command is sent during passive probing of auto serial candidates.
5. **Deterministic synthetic fault replay.** The P2 test iterates 20,000 wheel samples over 400 s at 50 Hz with additive noise, two LIO dropouts, two wheel-slip episodes, and older reference messages. It checks finite and bounded estimator state, stale-source covariance floor, and severe slip recognition. This is numerical regression, **not sensor accuracy or worst-case CPU timing on the F103**.

## Test commands

```bash
cd /media/sirobo/HDD/hoverboard-sideboard
tools/run_full_robustness_gate.sh
# Specific P1 ROS tests:
source /opt/ros/humble/setup.bash
PYTHONPATH=$PWD/ros2/sideboard_p2_fusion:$PYTHONPATH \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  ros2/sideboard_p2_fusion/test/test_p1_monotonic_and_persistence.py \
  ros2/sideboard_p2_fusion/test/test_p1_long_run_replay.py
```

## Residual risks requiring real hardware

- **MCU firmware size:** application 55 KiB partition (56,320 bytes) nearly full; APP_STLINK 56,296 B (24 B remaining), APP_STLINK_1KHZ 56,300 B (20 B remaining). Avoid additional Cortex-M3 code until flash is reclaimed or a larger MCU is chosen.
- **No live timing proof:** measure I2C bus occupancy, ESKF worst-case 5 ms / 10 ms service deadline, jitter P99/P99.9, stack high-water, STM32 watchdog resets, FIFO overrun/resync, UART ACK latency, and actual IMU sample age over 30–60 minutes.
- **No long-term absolute pose without aiding:** gyro/accelerometer integrated to XYZ/VXYZ drifts. Ground-truth RTK/LIO/UWB (or reliable wheel/steering) is required for bounded position/velocity after motion.
- **Simulated ROS clock reset:** out-of-order samples are rejected rather than silently rewinding; restart/reinitialization is currently required after a deliberate large backwards /clock reset. Implement an explicit guarded time-epoch reset before production use_sim_time replay workflows.
- **Passive identity is not authentication:** CRC+schema helps prevent wrong USB port selection but is not a cryptographic device identity; use udev serial/physical port binding and exclusive ownership on deployed robots.
- **No power-cut/fault injection of EEPROM or bootloader on physical MCU yet:** use a spare board/test rig before validation on moving vehicle.

## Next hardware-gated commissioning (operator present)

1. Confirm verified sideboard identity; avoid sending data to ESC, Android or the GNSS adapter. Use a safe bench with wheels unloaded.
2. Flash and test baseline APP_STLINK/APP_USART 500 Hz using documented bootloader workflow. Never change the flash partition map without backup.
3. Run `tools/imu/run_stm32_python_gate.sh --live 60` with board genuinely stationary, then 10–30-minute stationary logging and motion truth benchmarks.
4. For 1 kHz only on bench: flash an explicit `APP_STLINK_1KHZ` / `APP_USART_1KHZ` and run `tools/imu/run_stm32_python_gate.sh --live-1khz 60`, plus at least 30-minute thermal/bus/CPU stress.
5. Test controller outage while STATIONARY_ON, LiDAR dropout + slip, corrupted frames, USB re-enumeration, supply brownout, and settings power-cut.

No claim of physical robustness or positioning precision can be made without those measurements.

## P2: Long-run log continuity and fail-closed aiding (2026-10-08)

- Offline stationary CSV analyzer verifies each F0 sequence +1 modulo uint32, MCU epoch monotonic with bounded per-step gap, host monotonic P95/peak, and sensor age P95/peak. Real uint32 rollovers are accepted.
- All F5 health counters are checked per transition; a transient fault then counter reset cannot look healthy based on equal first/last counters. Contiguous diagnostic blackout >0.5 s fails.
- Stationary acceptance now bounds maximum position excursion, not just final minus initial position.
- Live stationary validation mirrors frame sequence/epoch continuity, host peak gap, sensor age peak, per-packet F5 health flags and counters, and requires F5 v2. Live hardware testing still pending.
- Serial aiding bridge rejects missing/stale/future wheel header timestamps, uses conservative 0.50 m/s sigma for invalid/unknown velocity variance, and expires the steering-valid permission after 0.25 s.
- Added 10 offline stream continuity tests and 7 P2 aiding fail-closed tests; both included in the gate.

Run: tools/run_full_robustness_gate.sh
Hardware profiling still required; F103 5ms/10ms deadline is not proven offline.
