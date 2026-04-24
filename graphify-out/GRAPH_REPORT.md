# Graph Report - D:\Code\bosch\Bezier_Embedded  (2026-04-24)

## Corpus Check
- 51 files · ~171,330 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 502 nodes · 1133 edges · 47 communities detected
- Extraction: 94% EXTRACTED · 6% INFERRED · 0% AMBIGUOUS · INFERRED: 63 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]
- [[_COMMUNITY_Community 7|Community 7]]
- [[_COMMUNITY_Community 8|Community 8]]
- [[_COMMUNITY_Community 9|Community 9]]
- [[_COMMUNITY_Community 10|Community 10]]
- [[_COMMUNITY_Community 11|Community 11]]
- [[_COMMUNITY_Community 12|Community 12]]
- [[_COMMUNITY_Community 13|Community 13]]
- [[_COMMUNITY_Community 14|Community 14]]
- [[_COMMUNITY_Community 15|Community 15]]
- [[_COMMUNITY_Community 16|Community 16]]
- [[_COMMUNITY_Community 17|Community 17]]
- [[_COMMUNITY_Community 18|Community 18]]
- [[_COMMUNITY_Community 19|Community 19]]
- [[_COMMUNITY_Community 20|Community 20]]
- [[_COMMUNITY_Community 21|Community 21]]
- [[_COMMUNITY_Community 22|Community 22]]
- [[_COMMUNITY_Community 23|Community 23]]
- [[_COMMUNITY_Community 24|Community 24]]
- [[_COMMUNITY_Community 25|Community 25]]
- [[_COMMUNITY_Community 26|Community 26]]
- [[_COMMUNITY_Community 27|Community 27]]
- [[_COMMUNITY_Community 28|Community 28]]
- [[_COMMUNITY_Community 29|Community 29]]
- [[_COMMUNITY_Community 30|Community 30]]
- [[_COMMUNITY_Community 31|Community 31]]
- [[_COMMUNITY_Community 32|Community 32]]
- [[_COMMUNITY_Community 33|Community 33]]
- [[_COMMUNITY_Community 34|Community 34]]
- [[_COMMUNITY_Community 35|Community 35]]
- [[_COMMUNITY_Community 36|Community 36]]
- [[_COMMUNITY_Community 37|Community 37]]
- [[_COMMUNITY_Community 38|Community 38]]
- [[_COMMUNITY_Community 39|Community 39]]
- [[_COMMUNITY_Community 40|Community 40]]
- [[_COMMUNITY_Community 41|Community 41]]
- [[_COMMUNITY_Community 42|Community 42]]
- [[_COMMUNITY_Community 43|Community 43]]
- [[_COMMUNITY_Community 44|Community 44]]
- [[_COMMUNITY_Community 45|Community 45]]
- [[_COMMUNITY_Community 46|Community 46]]

## God Nodes (most connected - your core abstractions)
1. `bno055_write_page_id()` - 177 edges
2. `bno055_set_operation_mode()` - 62 edges
3. `bno055_get_operation_mode()` - 60 edges
4. `applyInterpolatedCommand()` - 27 edges
5. `bno055_set_euler_unit()` - 21 edges
6. `bno055_get_euler_unit()` - 20 edges
7. `bno055_set_accel_unit()` - 19 edges
8. `bno055_set_gyro_unit()` - 19 edges
9. `bno055_get_accel_unit()` - 18 edges
10. `bno055_get_gyro_unit()` - 18 edges

## Surprising Connections (you probably didn't know these)
- `CMovelistexecutor()` --semantically_similar_to--> `Robot State Machine`  [INFERRED] [semantically similar]
  D:\Code\bosch\Bezier_Embedded\source\brain\movelistexecutor.cpp → source/brain/robotstatemachine.cpp
- `updatePoseEstimate()` --calls--> `hasValidYaw()`  [INFERRED]
  D:\Code\bosch\Bezier_Embedded\source\brain\movelistexecutor.cpp → D:\Code\bosch\Bezier_Embedded\source\periodics\imu.cpp
- `updatePoseEstimate()` --calls--> `getYawDegrees()`  [INFERRED]
  D:\Code\bosch\Bezier_Embedded\source\brain\movelistexecutor.cpp → D:\Code\bosch\Bezier_Embedded\source\periodics\imu.cpp
- `CImu()` --calls--> `setNewPeriod()`  [INFERRED]
  D:\Code\bosch\Bezier_Embedded\source\periodics\imu.cpp → source\utils\task.cpp
- `CImu()` --calls--> `bno055_set_operation_mode()`  [INFERRED]
  D:\Code\bosch\Bezier_Embedded\source\periodics\imu.cpp → source\drivers\bno055.cpp

## Hyperedges (group relationships)
- **Motion Control Pipeline** — robotstatemachine_crobotstatemachine, movelistexecutor_cmovelistexecutor, mpccontroller_cmpccontroller [INFERRED 0.85]
- **Power Supervision Loop** — powermanager_cpowermanager, instantconsumption_cinstantconsumption, totalvoltage_ctotalvoltage, serialtxbroker_cserialtxbroker [INFERRED 0.85]
- **Task Orchestration Runtime** — main_task_runtime, taskmanager_ctaskmanager, task_ctask, statereporter_cstatereporter, powermanager_cpowermanager [INFERRED 0.82]

## Communities

### Community 0 - "Community 0"
Cohesion: 0.04
Nodes (106): bno055_get_accel_any_motion_durn(), bno055_get_accel_any_motion_no_motion_axis_enable(), bno055_get_accel_any_motion_thres(), bno055_get_accel_bw(), bno055_get_accel_calib_stat(), bno055_get_accel_high_g_axis_enable(), bno055_get_accel_high_g_durn(), bno055_get_accel_high_g_thres() (+98 more)

### Community 1 - "Community 1"
Cohesion: 0.07
Nodes (56): getLinearSpeed(), resetTravelDistance(), serialCallbackENCTESTcommand(), activateDirectionSegment(), applyInterpolatedCommand(), clampFloat(), classifyDirection(), classifySpeedSample() (+48 more)

### Community 2 - "Community 2"
Cohesion: 0.07
Nodes (56): bno055_get_gyro_auto_sleep_durn(), bno055_get_operation_mode(), bno055_gyro_set_auto_sleep_durn(), bno055_set_accel_any_motion_durn(), bno055_set_accel_any_motion_no_motion_axis_enable(), bno055_set_accel_any_motion_thres(), bno055_set_accel_bw(), bno055_set_accel_high_g_axis_enable() (+48 more)

### Community 3 - "Community 3"
Cohesion: 0.11
Nodes (35): bno055_convert_double_euler_h_deg(), bno055_convert_double_euler_h_rad(), bno055_convert_double_euler_hpr_deg(), bno055_convert_double_euler_hpr_rad(), bno055_convert_double_euler_p_deg(), bno055_convert_double_euler_p_rad(), bno055_convert_double_euler_r_deg(), bno055_convert_double_euler_r_rad() (+27 more)

### Community 4 - "Community 4"
Cohesion: 0.11
Nodes (27): applyHampel(), applySpeedHysteresis(), CEncoder(), convertAngularToLinear(), ensureSensorConfigured(), getLastRejectedDeltaDegrees(), getLastRejectedLimitDegrees(), getMissingMeasurementDurationMs() (+19 more)

### Community 5 - "Community 5"
Cohesion: 0.15
Nodes (25): addConstraint(), appendCommandConstraints(), appendRateConstraints(), buildCondensedProblem(), buildCurrentState(), canAcceptIterationLimitedSolution(), clampFloat(), clearPreviousCorrection() (+17 more)

### Community 6 - "Community 6"
Cohesion: 0.09
Nodes (14): alertsCommand(), CAlerts(), _run(), serialCallbackIMUcommand(), CKlmanager(), serialCallbackKLCommand(), Task Runtime Wiring, CResourcemonitor() (+6 more)

### Community 7 - "Community 7"
Cohesion: 0.21
Nodes (22): bno055_convert_double_accel_x_mg(), bno055_convert_double_accel_x_msq(), bno055_convert_double_accel_xyz_mg(), bno055_convert_double_accel_xyz_msq(), bno055_convert_double_accel_y_mg(), bno055_convert_double_accel_y_msq(), bno055_convert_double_accel_z_mg(), bno055_convert_double_accel_z_msq() (+14 more)

### Community 8 - "Community 8"
Cohesion: 0.21
Nodes (22): bno055_convert_double_gyro_x_dps(), bno055_convert_double_gyro_x_rps(), bno055_convert_double_gyro_xyz_dps(), bno055_convert_double_gyro_xyz_rps(), bno055_convert_double_gyro_y_dps(), bno055_convert_double_gyro_y_rps(), bno055_convert_double_gyro_z_dps(), bno055_convert_double_gyro_z_rps() (+14 more)

### Community 9 - "Community 9"
Cohesion: 0.15
Nodes (6): Instant consumption task, Power safety and shutdown flow, Power Manager Task, Serial TX Broker, State reporter task, Total voltage task

### Community 10 - "Community 10"
Cohesion: 0.33
Nodes (3): Serial Subscriber Map, CMovelistexecutor(), Robot State Machine

### Community 11 - "Community 11"
Cohesion: 0.57
Nodes (7): bno055_convert_double_temp_celsius(), bno055_convert_double_temp_fahrenheit(), bno055_convert_float_temp_celsius(), bno055_convert_float_temp_fahrenheit(), bno055_get_temp_unit(), bno055_read_temp_data(), bno055_set_temp_unit()

### Community 12 - "Community 12"
Cohesion: 0.4
Nodes (1): CSerialMonitor()

### Community 13 - "Community 13"
Cohesion: 0.5
Nodes (1): CBatterymanager()

### Community 14 - "Community 14"
Cohesion: 0.5
Nodes (1): CBlinker()

### Community 15 - "Community 15"
Cohesion: 0.67
Nodes (0): 

### Community 16 - "Community 16"
Cohesion: 0.67
Nodes (3): bno055_convert_double_gravity_xyz_msq(), bno055_convert_float_gravity_xyz_msq(), bno055_read_gravity_xyz()

### Community 17 - "Community 17"
Cohesion: 0.67
Nodes (3): bno055_convert_double_linear_accel_y_msq(), bno055_convert_float_linear_accel_y_msq(), bno055_read_linear_accel_y()

### Community 18 - "Community 18"
Cohesion: 0.67
Nodes (3): bno055_convert_double_mag_y_uT(), bno055_convert_float_mag_y_uT(), bno055_read_mag_y()

### Community 19 - "Community 19"
Cohesion: 0.67
Nodes (3): bno055_convert_double_linear_accel_xyz_msq(), bno055_convert_float_linear_accel_xyz_msq(), bno055_read_linear_accel_xyz()

### Community 20 - "Community 20"
Cohesion: 0.67
Nodes (3): bno055_convert_gravity_double_x_msq(), bno055_convert_gravity_float_x_msq(), bno055_read_gravity_x()

### Community 21 - "Community 21"
Cohesion: 0.67
Nodes (3): bno055_convert_gravity_double_y_msq(), bno055_convert_gravity_float_y_msq(), bno055_read_gravity_y()

### Community 22 - "Community 22"
Cohesion: 0.67
Nodes (3): bno055_convert_double_mag_x_uT(), bno055_convert_float_mag_x_uT(), bno055_read_mag_x()

### Community 23 - "Community 23"
Cohesion: 0.67
Nodes (3): bno055_convert_gravity_double_z_msq(), bno055_convert_gravity_float_z_msq(), bno055_read_gravity_z()

### Community 24 - "Community 24"
Cohesion: 0.67
Nodes (3): bno055_convert_double_mag_z_uT(), bno055_convert_float_mag_z_uT(), bno055_read_mag_z()

### Community 25 - "Community 25"
Cohesion: 0.67
Nodes (3): bno055_convert_double_linear_accel_x_msq(), bno055_convert_float_linear_accel_x_msq(), bno055_read_linear_accel_x()

### Community 26 - "Community 26"
Cohesion: 0.67
Nodes (3): bno055_convert_double_mag_xyz_uT(), bno055_convert_float_mag_xyz_uT(), bno055_read_mag_xyz()

### Community 27 - "Community 27"
Cohesion: 0.67
Nodes (3): bno055_convert_double_linear_accel_z_msq(), bno055_convert_float_linear_accel_z_msq(), bno055_read_linear_accel_z()

### Community 28 - "Community 28"
Cohesion: 1.0
Nodes (3): Mbed OS cross-compilation setup, robot_car application target, mbed-tools compile command

### Community 29 - "Community 29"
Cohesion: 1.0
Nodes (1): CQueue

### Community 30 - "Community 30"
Cohesion: 1.0
Nodes (2): BFMC embedded platform project, Communication protocol between RPi and Nucleo

### Community 31 - "Community 31"
Cohesion: 1.0
Nodes (0): 

### Community 32 - "Community 32"
Cohesion: 1.0
Nodes (0): 

### Community 33 - "Community 33"
Cohesion: 1.0
Nodes (0): 

### Community 34 - "Community 34"
Cohesion: 1.0
Nodes (0): 

### Community 35 - "Community 35"
Cohesion: 1.0
Nodes (0): 

### Community 36 - "Community 36"
Cohesion: 1.0
Nodes (1): MPC Controller Design Notes

### Community 37 - "Community 37"
Cohesion: 1.0
Nodes (1): Fixed-size dense QP

### Community 38 - "Community 38"
Cohesion: 1.0
Nodes (1): Planner feedforward correction layer

### Community 39 - "Community 39"
Cohesion: 1.0
Nodes (1): Legacy units boundary

### Community 40 - "Community 40"
Cohesion: 1.0
Nodes (1): Directional segment logic

### Community 41 - "Community 41"
Cohesion: 1.0
Nodes (1): Zero-speed dwell bypass

### Community 42 - "Community 42"
Cohesion: 1.0
Nodes (1): Euler-discretized linear model

### Community 43 - "Community 43"
Cohesion: 1.0
Nodes (1): Projected gradient descent solver

### Community 44 - "Community 44"
Cohesion: 1.0
Nodes (1): Feedforward release fallback

### Community 45 - "Community 45"
Cohesion: 1.0
Nodes (1): Periodic task architecture

### Community 46 - "Community 46"
Cohesion: 1.0
Nodes (1): Notifications from Power Board

## Knowledge Gaps
- **19 isolated node(s):** `CEncoder`, `ISpeedingCommand`, `ISteeringCommand`, `CQueue`, `Task Runtime Wiring` (+14 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Community 29`** (2 nodes): `queue.hpp`, `CQueue`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 30`** (2 nodes): `BFMC embedded platform project`, `Communication protocol between RPi and Nucleo`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 31`** (1 nodes): `main.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 32`** (1 nodes): `globalsv.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 33`** (1 nodes): `BNO055.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 34`** (1 nodes): `globalsv.cpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 35`** (1 nodes): `queue.cpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 36`** (1 nodes): `MPC Controller Design Notes`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 37`** (1 nodes): `Fixed-size dense QP`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 38`** (1 nodes): `Planner feedforward correction layer`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 39`** (1 nodes): `Legacy units boundary`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 40`** (1 nodes): `Directional segment logic`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 41`** (1 nodes): `Zero-speed dwell bypass`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 42`** (1 nodes): `Euler-discretized linear model`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 43`** (1 nodes): `Projected gradient descent solver`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 44`** (1 nodes): `Feedforward release fallback`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 45`** (1 nodes): `Periodic task architecture`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 46`** (1 nodes): `Notifications from Power Board`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `_run()` connect `Community 3` to `Community 1`?**
  _High betweenness centrality (0.322) - this node is a cross-community bridge._
- **Why does `getLinearSpeed()` connect `Community 1` to `Community 3`, `Community 4`?**
  _High betweenness centrality (0.288) - this node is a cross-community bridge._
- **Why does `applyInterpolatedCommand()` connect `Community 1` to `Community 5`?**
  _High betweenness centrality (0.193) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `bno055_set_operation_mode()` (e.g. with `configureSensor()` and `CImu()`) actually correct?**
  _`bno055_set_operation_mode()` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 6 inferred relationships involving `applyInterpolatedCommand()` (e.g. with `solve()` and `getLinearSpeed()`) actually correct?**
  _`applyInterpolatedCommand()` has 6 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `bno055_set_euler_unit()` (e.g. with `configureSensor()` and `CImu()`) actually correct?**
  _`bno055_set_euler_unit()` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `CEncoder`, `ISpeedingCommand`, `ISteeringCommand` to the rest of the system?**
  _19 weakly-connected nodes found - possible documentation gaps or missing edges._