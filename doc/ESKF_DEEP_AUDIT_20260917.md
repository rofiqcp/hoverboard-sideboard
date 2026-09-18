# ESKF Deep Audit — STM32F103C8T6 — 2026-09-17

## Scope
Audit covers MPU6xxx FIFO/timing, sensor calibration, sensor-to-body mounting, IMU preintegration, 15-state ESKF nominal propagation, covariance propagation, gravity/ZUPT/zero-rate/external-aid fusion, observability, EEPROM persistence, protocol timing, and STM32F103C8T6 CPU/RAM constraints.

## Frame and state convention
- Nominal quaternion `q`: body -> local world.
- Body frame intended for ROS/AGV: +X forward, +Y left, +Z up.
- Local world uses +Z up. Yaw positive CCW around +Z.
- Error state: `[dtheta,dvel,dpos,dbg,dba]`, 15 states.
- Right attitude error: `q_true = q_nominal * dq`.
- Velocity and position states refer to the IMU point unless an external measurement is explicitly lever-arm corrected.

## What is structurally correct
- Right-error attitude propagation/Jacobians are sign-consistent.
- `R` is body->world; gravity prediction is `R^T * ez`.
- State mechanization subtracts residual gyro/accel bias from preintegrated increments.
- Mid-interval attitude is used for delta-velocity rotation.
- Position uses trapezoidal velocity integration.
- Coning correction follows the ArduPilot streaming form.
- Static calibration is performed in sensor frame before sensor->body rotation.
- Gravity does not directly observe yaw; ZUPT restricts poorly observable accel-bias components.
- Wheel/body velocity Jacobian has been finite-difference checked for the right-error convention.
- Measurement covariance update uses the Joseph-equivalent scalar form even when K is projected/scaled.

## P0 findings proven by regression
1. `eskf_nav_predict_delta()` clamps dt to 30 ms while retaining full delta-angle/delta-velocity. A 40 ms stationary delta produces +0.0980665 m/s false vertical velocity. State propagation must use the real preintegrated dt or reject the whole interval.
2. Gravity fusion currently uses the last raw/calibrated sample, while propagation uses a 100 Hz integrated average. Synthetic +/-0.1 g 200 Hz X acceleration averages to zero in propagation but aliases to a constant gravity-direction error, producing ~5.7 deg false pitch and multi-m/s velocity drift. Gravity measurement must use `delta_velocity / delta_dt` and clipping/dynamic gates.
3. Covariance propagation uses `P += dt(AP+PA^T)+Q`, omitting `dt^2 A P A^T`. More critically, individual diagonal entries are clamped to 1e6 while off-diagonals keep growing. This can make P indefinite while `eskf_nav_is_healthy()` still returns true because the same upper clamp masks the health threshold.
4. Filter health recovery reinitializes tilt from the instantaneous accelerometer even while moving. A covariance-only fault during acceleration can therefore create a false tilt. Recovery needs covariance repair/reset first; full tilt reinitialization is allowed only when stationary or externally constrained.

## P1 estimator improvements
- Use sparse `F P F^T + Qd` with `F ~= I + A dt`; add discrete accel-noise velocity/position cross terms.
- Replace diagonal-only upper clamp by block-consistent covariance conditioning and Cauchy/finite checks.
- Gate gravity on averaged specific force; suppress/deweight it during fresh horizontal aiding and reject IMU clipping.
- Pre-gate vector measurements before sequential mutation, or report/track partial-axis fusion explicitly.
- Add source-specific aiding timeout and delay compensation. Absolute timestamps must be in MCU clock domain; otherwise protocol should carry measurement age or synchronize host<->MCU clocks.
- Define external world frame contract explicitly before ROS/GNSS fusion. World position/velocity must use the same Z-up local frame and heading alignment as the ESKF.
- Clarify external measurement reference point. Wheel aiding is converted body-origin -> IMU point via `omega x r`; world position/velocity currently assume the supplied value already refers to the ESKF/IMU point.
- Add NHC slip gating/noise scaling versus yaw rate/lateral acceleration/wheel consistency.
- Reset observed FIFO sample-rate estimator after FIFO reset, as production FIFO backends do.
- Add raw accel/gyro clipping flags/counters and suppress gravity fusion during clipping.
- Mark velocity/position validity from aiding age and covariance, not merely `dead_reckoning`.

## Calibration audit
### Still / gyro bias
- Correctly estimates sensor-frame gyro bias and noise before mounting.
- Transactional RAM commit is implemented.
- Motion resets the accumulator.
- Recommended: persist only after flash save verifies; on save failure keep retry-needed state instead of clearing it.

### Accelerometer six-face
Current runtime model is `a_corr = T3x3 * (a_raw - offset)` and is computationally appropriate for F103 (9 multiplies/sample).
- Pair centers estimate offset.
- Pair differences form a 3x3 sensitivity/cross-axis matrix, then firmware inverts it.
- Residual and range gates are present.
Remaining limitations:
- Six centroids rely on accurate physical face placement and provide weak protection against orientation-placement error.
- Full arbitrary 3x3 can absorb some placement rotation/cross-axis effects.
- No condition-number gate beyond determinant/column norms.
Recommended production calibration: collect more than six distinct static orientations on host, fit a constrained ellipsoid/symmetric correction, validate on held-out orientations, then upload the 3x3 runtime matrix. Keep six-face as simple field calibration fallback.

### Mounting
Mounting quaternion is separate and correctly applied after sensor-frame calibration. Calibration instructions must clearly distinguish sensor/PCB axes from vehicle body axes. Config changes that require attitude reinit should be allowed only while stationary.

### Thermal
Runtime linear thermal correction and reference-temperature rebasing are coherent, but there is no automated thermal learning procedure. Add a logger/fitter using multiple stationary temperature plateaus. Gyro and accel slopes must be fit in sensor frame before mounting.

### Lever arm
`imu_position_body` and wheel `omega x r` correction are correct for an ESKF state located at the IMU. External position/yaw/GNSS interfaces need equally explicit reference-point handling.

### Noise/Q/R
Still calibration stores standard deviations but does not identify continuous gyro/accel noise density or bias random walk. Add stationary logging + Allan deviation on the host. Use the result to derive `gyro_noise`, `accel_process_noise`, `gyro_bias_walk`, and `accel_bias_walk`; do not equate a short-window sample standard deviation directly with random-walk density.

## Persistence
EEPROM settings are CRC protected but still single-page erase/program. Power loss during calibration save can lose calibration (not brick firmware). Recommended: two-page A/B journal, generation counter, CRC, commit marker, choose newest valid record on boot.

## F103C8T6 resource audit
Current APP build: ~42.6 kB flash (~65%), static RAM ~1.9 kB plus reserved heap/stack; Cortex-M3 uses software floating point. Keep the 15x15 covariance but exploit sparsity. Avoid full generic matrix inversion/multiplication in the 100 Hz path. A sparse `dt^2 A P A^T` update is feasible; A has nonzero rows only for attitude/velocity/position.

## Validation required after P0 patches
- dt 10/20/40 ms stationary propagation: zero velocity artifact.
- 200 Hz +/-horizontal acceleration alias test: no false gravity tilt when 100 Hz delta average is zero.
- covariance symmetry + minimum eigenvalue stress tests.
- stationary 10 min, yaw drift, velocity/position bounded with ZUPT.
- moving replay with wheel/NHC/yaw aiding; innovation/NIS/reject counters.
- FIFO overflow/resync/gap tests.
- six-face synthetic + physical calibration; transactional failure and EEPROM power-loss tests.

## Implemented during this audit (not yet committed)
- Exact preintegrated dt is now used up to 50 ms; no independent 30 ms clamp.
- Covariance prediction now includes sparse `dt^2*A*P*A^T` and discrete accel-noise v/p cross terms.
- Removed the covariance upper-diagonal clamp that could hide an indefinite P; health uses finite/positive/Cauchy checks.
- Gravity correction now consumes averaged `delta_velocity/dt`, not the last 200 Hz sample.
- Added gravity-direction gates (4 deg stationary, 3 deg moving) to reject sustained horizontal acceleration masquerading as tilt.
- Stillness detector now checks gravity-direction agreement using dot-product thresholds without adding a sqrt in the 100 Hz loop.
- FIFO observed-rate measurement window resets on FIFO reset/resync.
- Six-face collection now requires 50 stable samples on one face before centroid accumulation; matrix condition gets a Frobenius condition gate.
- Estimator recovery keeps a last known-good nominal state and resets covariance instead of reinitializing tilt from instantaneous acceleration.
- Mount/thermal/lever configuration changes are rejected while the board is not stationary.
- Permanent regression now covers legal 40 ms FIFO deltas, covariance invariants for 120 s, covariance-only recovery, sustained 0.1 g acceleration, source reset/rejection, and full 3x3 calibration.

Hardware after these changes: 500/500 telemetry frames valid, 0 sequence loss, 0 CRC/framing errors, 50.0 Hz stream, health reset count 0.

## Resolution status — implemented and hardware-validated
- P0 dt mismatch fixed: full preintegrated delta always uses the same real `dt`; legal intervals up to 50 ms are accepted, otherwise the whole delta is rejected.
- Gravity fusion uses averaged `delta_velocity/dt` plus magnitude+direction gates; sustained 0.1 g horizontal acceleration regression no longer becomes false pitch.
- Covariance uses sparse `F P F^T + Qd`, preserves symmetry/PSD in the regression set, and no longer hides failure behind an upper diagonal clamp.
- Health recovery restores last-known-good nominal state and resets covariance instead of rebuilding tilt from moving accelerometer data.
- Still detector includes gravity-direction hysteresis. FIFO observed-rate window resets after resync.
- Six-face collection requires stable dwell and condition-number gating. A host multi-orientation ellipsoid fitter is provided for higher-accuracy calibration.
- Settings persistence is now A/B journaled: A=`0x0800F400`, manifest=`0x0800F800`, B=`0x0800FC00`, with generation+CRC+commit-last. Power-loss/failover was tested by erasing the newest copy and booting from the older committed copy.
- Calibration save is transactional through persistence: if flash commit fails, firmware reloads the last committed settings and reports calibration failure instead of running an unpersisted candidate.
- External aiding protocol has explicit frame and timing modes. ENU position/velocity is rejected until ENU yaw alignment exists; any full estimator re-init clears that alignment.
- Telemetry v5 exposes firmware-converted temperature, WHO_AM_I, IMU class, and observed FIFO sample-rate for host thermal/noise fitting.
- Host tools added: `imu_log.py`, `thermal_calibration.py`, `allan_analysis.py`, `accel_ellipsoid_calibration.py`. Short/noisy/poor-coverage datasets are rejected.
- Explicit sculling correction was **not** added: current ArduPilot raw-accel backend integrates accel*dt while applying coning to delta-angle, and this firmware already rotates delta-V with mid-interval attitude. A custom extra sculling term would risk double compensation without a matching validated reference implementation.

Final hardware gate after these changes: still calibration `DONE`, telemetry 50.0 Hz, zero sequence/CRC/framing errors, health resets 0, WHO_AM_I `0x72`, observed sensor rate ~200.5 Hz.

## Follow-up hardware validation — 2026-09-18
- Fixed bootloader command latency/RX-overrun: application CRC validity is cached once at boot; INFO/GO are O(1), ERASE invalidates the cache, VERIFY commits it, and TX-complete is awaited before GO.
- Bootloader validation: 10/10 app-F1-INFO-GO cycles plus full 46.9 kB UART rewrite/CRC/GO passed. Settings journal A/B SHA remained byte-identical across firmware update.
- F4 aiding v2 adds 16-bit request ID and firmware duplicate-reply cache. Retry of the same ID never fuses the measurement twice. Hardware duplicate test: first ACK intentionally/lossy, retry received status 4, reject counter delta remained exactly 1.
- F4 stress after the change: stale/rejected responses 50/50, accepted wheel/NHC responses 20/20, no timeout.
- Real still calibration reached DONE and persisted to journal; newest journal A generation 13 was CRC-valid, backup B generation 12 remained CRC-valid. `flags=0x1`, so no synthetic rotate/mount/thermal validity was written.
- Incomplete rotate + finish failed with error 2; rotate + cancel returned IDLE; both left journal SHA unchanged. Singular accel-cal command was rejected with status 2 and also left both journal pages unchanged.
- Telemetry final gate: 1000/1000 frames, zero sequence/CRC/framing errors, ~49.9 Hz, health reset 0. Native VESC 20/20, CONFIG GET 10/10, calibration STATUS 10/10.
- Allan tool sample-rate calculation now uses unwrapped MCU `board_us`. A real 25 s stationary log measured 49.942 Hz; bias-walk estimation remains intentionally disabled until >=10 min data is available.
- Physical six-face/multi-orientation, thermal-plateau and lever-arm *values* are not fabricated. Their state machines, rejection gates, persistence and host synthetic fits are tested; actual values still require physically moving/heating/measuring the board.
