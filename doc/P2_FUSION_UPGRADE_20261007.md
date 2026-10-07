# P2 NUC Fusion Upgrade — 2026-10-07

## Scope

P2 memindahkan estimasi yang terlalu berat untuk STM32F103C8 ke NUC ROS2. Firmware P0/P1 tetap menjadi estimator lokal cepat, sedangkan NUC menangani online calibration, slip confidence lanjutan, delayed fusion, LiDAR odometry, dan global GNSS/RTK anchoring.

Alasan arsitektur ini:
- aplikasi F103 hanya memiliki sekitar 308 byte headroom setelah P1;
- full history covariance 15x15 + rewind/repropagation tidak layak dipaksakan ke F103C8;
- ROS2 NUC memiliki RAM/CPU cukup untuk delayed-state smoothing dan multi-sensor fusion.

## Arsitektur akhir

```text
STM32 sideboard
  MPU6xxx 500 Hz -> preintegration -> local ESKF 100 Hz
  wheel/NHC -> current NIS -> MCU slip observer
             |
             | serial / ROS wheel source
             v
NUC P2
  raw wheel odom ----+
  steering encoder --+--> sideboard_p2_fusion
  FAST-LIO /Odometry+       |
                             +--> online wheel scale
                             +--> steering-zero + wheelbase calibration
                             +--> acceleration-consistency evidence
                             +--> signed slip ratio
                             +--> DOB-inspired slip velocity
                             +--> adaptive wheel covariance
                             |
                             +--> /p2/wheel/odometry
                             +--> serial aiding bridge -> STM32 F4 protocol v2
                             |
                             v
                    robot_localization EKF local
                    smooth_lagged_data + history
                             |
                    /odometry/local
                             |
              GNSS/RTK + ENU absolute heading
                             |
                    navsat_transform
                             |
                    /odometry/gps
                             |
                    robot_localization EKF global
                             |
                    /odometry/global
```

## ROS2 package

Package:
`ros2/sideboard_p2_fusion`

Main executables:
- `p2_fusion_node`
- `serial_aiding_bridge`

Main launch files:
- `p2_fusion.launch.py`
- `mid360_p2_stack.launch.py`

Helper scripts:
- `tools/p2/build_p2_stack.sh`
- `tools/p2/run_p2_core.sh`
- `tools/p2/run_mid360_p2.sh`

## 1. Online wheel scale / effective radius

Model:

```text
v_ref = scale * v_wheel_raw
```

Scalar bounded RLS estimates `scale`.

Default constraints:
- scale range: 0.75 .. 1.25
- reference must be fresh
- |speed| >= 0.60 m/s
- low yaw-rate for scale update
- low acceleration
- speed discrepancy <= 12%
- no strong external slip evidence

This prevents tire slip from being incorrectly learned as a permanent wheel-radius correction.

Synthetic regression:
- raw wheel: 2.50 m/s
- LIO reference: 2.30 m/s
- expected scale: 0.92
- converged result: ~0.9200004

## 2. Steering zero + wheelbase online correction

Ackermann model:

```text
yaw_rate_pred = v / L * tan(delta_measured - delta_zero)
```

P2 estimates:
- `delta_zero`: steering encoder zero offset
- `L`: effective wheelbase

Updates are quality-gated by:
- minimum vehicle speed
- steering excitation
- low acceleration
- wheel/reference speed agreement
- no external slip indication

Synthetic truth:
- wheelbase: 1.20 m
- steering zero: 0.04 rad

From intentionally wrong initial state:
- wheelbase initial: 1.00 m
- steering zero initial: 0 rad

Observed convergence:
- wheelbase ~1.184 m after 5000 samples
- steering zero ~0.0390 rad
- 7000-sample regression requires <0.03 m and <0.008 rad error.

## 3. Longitudinal slip probability

Evidence sources:
- wheel/reference signed speed ratio;
- acceleration mismatch;
- Ackermann yaw-rate mismatch when steering is fresh;
- optional external MCU slip ratio.

Signed ratio:

```text
slip_ratio = (v_wheel_corrected - v_reference)
             / max(|v_wheel_corrected|, |v_reference|, v_min)
```

Positive:
- wheel spin / wheel reports faster than reference.

Negative:
- braking skid / wheel reports slower than reference.

P2 uses max-dominant evidence rather than forcing all evidence to corroborate. A pure longitudinal wheel-spin can therefore reach severe-slip state even if yaw evidence is quiet.

Regression:
- wheel 7 m/s vs LIO 5 m/s -> slip probability ~0.9995
- severe slip = true
- wheel sigma -> 5.0 m/s
- online scale remains unchanged.

## 4. Disturbance-observer-inspired slip velocity

Residual:

```text
v_slip = LPF(v_wheel_corrected - v_LIO)
```

Default observer bandwidth:
- 3 Hz

Bound:
- +/-8 m/s

This is not presented as a full nonlinear InEKF slip-state estimator. It is a compact disturbance observer suitable for adaptive measurement confidence and diagnostics.

## 5. Adaptive covariance

Base wheel sigma:
- 0.05 m/s

Sigma grows with:
- slip probability squared;
- absolute DOB slip velocity.

Maximum:
- 5.0 m/s

When slip probability >= 0.92:
- wheel measurement is effectively strongly deweighted;
- P2 odometry covariance is inflated;
- MCU serial bridge sends the increased sigma as well.

## 6. Acceleration consistency

Wheel acceleration and LIO-derived forward acceleration are independently differentiated and filtered.

Large mismatch:
- raises slip probability;
- raises covariance;
- blocks online calibration during high dynamics.

This prevents wheel radius / geometry calibration from learning during hard acceleration or braking.

## 7. Steering freshness safety

Steering input timeout:
- 0.25 s by default.

When steering is stale:
- geometry learning is disabled;
- yaw mismatch evidence is disabled;
- yaw-rate covariance becomes very large;
- serial bridge sends the backward-compatible 17-byte wheel packet;
- `AID_FLAG_EXPECTED_YAW_RATE` is not set.

Therefore stale steering cannot masquerade as a valid expected yaw-rate of zero.

## 8. Delayed fusion on the NUC

Local EKF uses:
- `smooth_lagged_data: true`
- `history_length: 1.0`
- `permit_corrected_publication: true`
- `predict_to_current_time: true`

Sources:
- `/p2/wheel/odometry`: vx, lateral NHC, expected yaw-rate
- `/p2/lio/odometry`: canonicalized FAST-LIO x, y, yaw in `odom/base_link`

This is the full delayed-measurement solution that was intentionally not put on the F103.

## 9. GNSS / RTK global fusion

Global layer:
- local odometry: `/odometry/local`
- GNSS metric odometry from navsat_transform: `/odometry/gps`
- result: `/odometry/global`

Safety requirement:
- GNSS mode requires a world-referenced ENU heading.
- default IMU topic is `/imu/enu_absolute`.
- `use_odometry_yaw` defaults to false.

Do not feed bare MPU6xxx integrated yaw as an absolute ENU heading.

## 10. Serial aiding back to STM32

`serial_aiding_bridge` sends P2-corrected wheel data using the existing VESC F4 v2 protocol.

Fresh steering:
- 21-byte payload
- NHC flag
- expected yaw-rate flag/value

Stale steering:
- 17-byte backward-compatible payload
- NHC only

Bridge behavior:
- auto-detect by-id / ttyUSB / ttyACM
- 921600 baud
- max packet age 14 ms
- MCU P1 max age is 15 ms
- incremental request IDs
- ACK parser
- reconnect without crashing the fusion stack
- diagnostics for sent/ACK/reject/stale/IO error counts.

## 11. FAST-LIO ROS2 integration fixes

Repository:
`/media/sirobo/HDD/slam3d/FAST_LIO_SAM_ROS2`

ROS2 package name is:
`fast_lio_sam`

Its launch file incorrectly referenced `fast_lio` in two places.

Fixed:
- `get_package_share_directory('fast_lio_sam')`
- `Node(package='fast_lio_sam', ...)`

After fix:
`ros2 launch fast_lio_sam mapping.launch.py rviz:=false config_file:=mid360.yaml`
starts `fastlio_mapping` successfully.

FAST-LIO publishes raw `/Odometry` as `camera_init -> body`. P2 therefore canonicalizes this stream before the local EKF:
- raw input: `/Odometry`, `camera_init/body`;
- configurable planar `body -> base_link` extrinsic;
- canonical output: `/p2/lio/odometry`, `odom/base_link`;
- position/yaw covariance floors prevent zero-confidence startup measurements.

The FAST-LIO source also originally called `publish()` before filling the current covariance matrix. P2 integration fixes this order so pose covariance belongs to the same state/timestamp that is published.

The old CMake cache also mixed:
- /usr/local PCL 1.15
- ROS Humble/system PCL 1.12

FAST-LIO was clean-rebuilt with:

```bash
-DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl
```

Final `ldd` shows only PCL 1.12:
- libpcl_common.so.1.12
- libpcl_io.so.1.12
- libpcl_io_ply.so.1.12

## 12. Test results

Pure P2 unit/integration:
- 17 tests PASS
- 0 failures

Covered:
- wheel scale convergence
- scale freeze during strong slip
- wheelbase/steering-zero convergence
- positive wheel spin
- negative braking skid
- acceleration mismatch covariance
- stale LIO handling
- stale steering handling
- persistence restore
- VESC 21-byte and 17-byte layouts
- parser resynchronization
- delayed EKF config
- GNSS global anchor config
- absolute ENU heading default

ROS synthetic smoke:
- 50 Hz synthetic stream
- raw wheel: 2.08 m/s
- LIO reference: 2.00 m/s
- raw LIO frame intentionally tested as `camera_init/body`
- canonical LIO messages: 199 in `odom/base_link`
- P2 wheel messages: 199
- local EKF messages: 199
- corrected P2 vx: ~2.0000 m/s

Full-stack no-device smoke:
- livox_ros_driver2 starts
- fastlio_mapping starts
- P2 fusion starts
- serial bridge starts
- local EKF starts

With serial absent:
- bridge remains alive
- diagnostic state = WARN
- message = `waiting for sideboard serial`

With wheel data absent:
- P2 remains alive
- diagnostic state = WARN
- message = `waiting for wheel odometry`

GNSS-mode smoke:
- local EKF starts
- navsat_transform starts
- global EKF starts
- no parameter/launch errors.

## 13. Current hardware limitation

At validation time:
- NUC interface `enp4s0` had `192.168.1.10/24`.
- MID360s config expected LiDAR `192.168.1.197`.
- Livox driver parsed config and initialized successfully.
- hardware later became unreachable at ARP level;
- `/livox/lidar` and `/livox/imu` were not publishing.

Therefore P2 software, ROS graph, FAST-LIO integration, delayed fusion, calibration logic, and failure behavior are validated, but live Mid360 motion data is not yet validated in this session.

The sideboard USB/serial is also not currently attached, so corrected P2 aiding has not yet been ACK-tested by the physical STM32.

## Build / run

Build:

```bash
cd /media/sirobo/HDD/hoverboard-sideboard
tools/p2/build_p2_stack.sh
```

Run only P2 core:

```bash
tools/p2/run_p2_core.sh
```

Run Mid360 + FAST-LIO + P2:

```bash
tools/p2/run_mid360_p2.sh
```

Enable serial aiding when sideboard is physically connected:

```bash
tools/p2/run_mid360_p2.sh enable_serial_bridge:=true
```

Enable GNSS/RTK only after an ENU absolute heading source is available:

```bash
tools/p2/run_mid360_p2.sh \
  enable_gnss:=true \
  imu_topic:=/imu/enu_absolute \
  gps_topic:=/fix
```

## Production validation still required

When hardware is connected:
1. confirm `/livox/lidar` and `/livox/imu` rates;
2. verify FAST-LIO `/Odometry` timestamp latency;
3. verify sideboard serial ACK age is consistently <15 ms;
4. straight runs at several speeds for wheel-scale convergence;
5. left/right repeated constant-radius turns for steering-zero/wheelbase;
6. hard acceleration/braking;
7. controlled wheel spin/skid;
8. compare `/odometry/local` against FAST-LIO and RTK ground truth;
9. verify no calibration movement while slip probability is high;
10. validate map/odom/ENU frame alignment before enabling global GNSS fusion.

## Status

P2 software implementation: COMPLETE.

P2 ROS2 build/test/synthetic runtime: COMPLETE.

FAST-LIO ROS2 launch and PCL ABI cleanup: COMPLETE.

Live Mid360 data validation: BLOCKED by current LiDAR reachability.

Physical sideboard serial ACK validation: BLOCKED because sideboard serial is not attached.
