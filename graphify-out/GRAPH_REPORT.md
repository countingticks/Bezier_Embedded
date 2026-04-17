# Graph Report - D:/Code/bosch/Bezier_Embedded  (2026-04-16)

## Corpus Check
- 56 files · ~129,144 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 539 nodes · 1224 edges · 32 communities detected
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 91 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_LinearMag Conversions|Linear/Mag Conversions]]
- [[_COMMUNITY_BNO055 Motion Config|BNO055 Motion Config]]
- [[_COMMUNITY_Periodic Task Headers|Periodic Task Headers]]
- [[_COMMUNITY_Move Executor Logic|Move Executor Logic]]
- [[_COMMUNITY_Test Command Hooks|Test Command Hooks]]
- [[_COMMUNITY_Euler Angle Conversions|Euler Angle Conversions]]
- [[_COMMUNITY_Encoder Filtering|Encoder Filtering]]
- [[_COMMUNITY_MPC Solver Assembly|MPC Solver Assembly]]
- [[_COMMUNITY_Alerts Task|Alerts Task]]
- [[_COMMUNITY_Accel Conversions|Accel Conversions]]
- [[_COMMUNITY_Gyro Conversions|Gyro Conversions]]
- [[_COMMUNITY_MPC Design Rationale|MPC Design Rationale]]
- [[_COMMUNITY_Temperature Handling|Temperature Handling]]
- [[_COMMUNITY_Gyro Config|Gyro Config]]
- [[_COMMUNITY_Battery Manager|Battery Manager]]
- [[_COMMUNITY_Blinker Task|Blinker Task]]
- [[_COMMUNITY_Component Generator|Component Generator]]
- [[_COMMUNITY_Linear Accel X|Linear Accel X]]
- [[_COMMUNITY_Magnetometer X|Magnetometer X]]
- [[_COMMUNITY_Gravity Y|Gravity Y]]
- [[_COMMUNITY_Gravity XYZ|Gravity XYZ]]
- [[_COMMUNITY_Gravity X|Gravity X]]
- [[_COMMUNITY_Linear Accel Z|Linear Accel Z]]
- [[_COMMUNITY_Magnetometer Y|Magnetometer Y]]
- [[_COMMUNITY_Build & Compile|Build & Compile]]
- [[_COMMUNITY_Queue API|Queue API]]
- [[_COMMUNITY_Project Overview|Project Overview]]
- [[_COMMUNITY_Main Header|Main Header]]
- [[_COMMUNITY_Global State Decls|Global State Decls]]
- [[_COMMUNITY_BNO055 Header|BNO055 Header]]
- [[_COMMUNITY_Global State Storage|Global State Storage]]
- [[_COMMUNITY_Queue Implementation|Queue Implementation]]

## God Nodes (most connected - your core abstractions)
1. `bno055_write_page_id()` - 177 edges
2. `bno055_set_operation_mode()` - 61 edges
3. `bno055_get_operation_mode()` - 60 edges
4. `applyInterpolatedCommand()` - 27 edges
5. `bno055_set_euler_unit()` - 20 edges
6. `bno055_set_accel_unit()` - 19 edges
7. `bno055_set_gyro_unit()` - 19 edges
8. `bno055_get_euler_unit()` - 19 edges
9. `bno055_get_accel_unit()` - 18 edges
10. `bno055_get_gyro_unit()` - 18 edges

## Surprising Connections (you probably didn't know these)
- `MPC Controller Design Notes` --references--> `CMovelistexecutor()`  [EXTRACTED]
  docs/mpc_controller.md → source\brain\movelistexecutor.cpp
- `Legacy units boundary` --rationale_for--> `CMovelistexecutor()`  [EXTRACTED]
  docs/mpc_controller.md → source\brain\movelistexecutor.cpp
- `Directional segment logic` --rationale_for--> `CMovelistexecutor()`  [EXTRACTED]
  docs/mpc_controller.md → source\brain\movelistexecutor.cpp
- `Zero-speed dwell bypass` --rationale_for--> `CMovelistexecutor()`  [EXTRACTED]
  docs/mpc_controller.md → source\brain\movelistexecutor.cpp
- `CRobotStateMachine()` --semantically_similar_to--> `CMovelistexecutor()`  [INFERRED] [semantically similar]
  source\brain\robotstatemachine.cpp → source\brain\movelistexecutor.cpp

## Hyperedges (group relationships)
- **Motion Control Pipeline** — robotstatemachine_crobotstatemachine, movelistexecutor_cmovelistexecutor, mpccontroller_cmpccontroller [INFERRED 0.85]
- **Power Supervision Loop** — powermanager_cpowermanager, instantconsumption_cinstantconsumption, totalvoltage_ctotalvoltage, serialtxbroker_cserialtxbroker [INFERRED 0.85]
- **Task Orchestration Runtime** — main_task_runtime, taskmanager_ctaskmanager, task_ctask, statereporter_cstatereporter, powermanager_cpowermanager [INFERRED 0.82]

## Communities

### Community 0 - "Linear/Mag Conversions"
Cohesion: 0.03
Nodes (119): bno055_convert_double_linear_accel_xyz_msq(), bno055_convert_double_linear_accel_y_msq(), bno055_convert_double_mag_xyz_uT(), bno055_convert_double_mag_z_uT(), bno055_convert_float_linear_accel_xyz_msq(), bno055_convert_float_linear_accel_y_msq(), bno055_convert_float_mag_xyz_uT(), bno055_convert_float_mag_z_uT() (+111 more)

### Community 1 - "BNO055 Motion Config"
Cohesion: 0.08
Nodes (52): bno055_get_operation_mode(), bno055_set_accel_any_motion_durn(), bno055_set_accel_any_motion_no_motion_axis_enable(), bno055_set_accel_any_motion_thres(), bno055_set_accel_bw(), bno055_set_accel_high_g_axis_enable(), bno055_set_accel_high_g_durn(), bno055_set_accel_high_g_thres() (+44 more)

### Community 2 - "Periodic Task Headers"
Cohesion: 0.06
Nodes (32): CInstantConsumption(), _run(), serialCallbackINSTANTcommand(), void_InstantSafetyMeasure(), Power safety and shutdown flow, CPowermanager(), _run(), Notifications from Power Board (+24 more)

### Community 3 - "Move Executor Logic"
Cohesion: 0.12
Nodes (44): getLinearSpeed(), activateDirectionSegment(), applyInterpolatedCommand(), clampFloat(), classifyDirection(), classifySpeedSample(), computeHorizonSeedProgressMm(), computeMatchedProgressMm() (+36 more)

### Community 4 - "Test Command Hooks"
Cohesion: 0.07
Nodes (28): resetTravelDistance(), serialCallbackENCTESTcommand(), serialCallbackIMUcommand(), CKlmanager(), serialCallbackKLCommand(), CResourcemonitor(), _run(), serialCallbackRESMONCommand() (+20 more)

### Community 5 - "Euler Angle Conversions"
Cohesion: 0.11
Nodes (30): bno055_convert_double_euler_h_deg(), bno055_convert_double_euler_h_rad(), bno055_convert_double_euler_hpr_deg(), bno055_convert_double_euler_hpr_rad(), bno055_convert_double_euler_p_deg(), bno055_convert_double_euler_p_rad(), bno055_convert_double_euler_r_deg(), bno055_convert_double_euler_r_rad() (+22 more)

### Community 6 - "Encoder Filtering"
Cohesion: 0.13
Nodes (21): applyHampel(), applySpeedHysteresis(), CEncoder(), convertAngularToLinear(), ensureSensorConfigured(), getRawAngleDegrees(), getTotalDisplacementDegrees(), getTravelDistanceMm() (+13 more)

### Community 7 - "MPC Solver Assembly"
Cohesion: 0.16
Nodes (24): addConstraint(), appendCommandConstraints(), appendRateConstraints(), buildCondensedProblem(), buildCurrentState(), canAcceptIterationLimitedSolution(), clampFloat(), clearPreviousCorrection() (+16 more)

### Community 8 - "Alerts Task"
Cohesion: 0.1
Nodes (15): alertsCommand(), CAlerts(), _run(), loop(), main(), setup(), Task Runtime Wiring, Periodic task architecture (+7 more)

### Community 9 - "Accel Conversions"
Cohesion: 0.21
Nodes (22): bno055_convert_double_accel_x_mg(), bno055_convert_double_accel_x_msq(), bno055_convert_double_accel_xyz_mg(), bno055_convert_double_accel_xyz_msq(), bno055_convert_double_accel_y_mg(), bno055_convert_double_accel_y_msq(), bno055_convert_double_accel_z_mg(), bno055_convert_double_accel_z_msq() (+14 more)

### Community 10 - "Gyro Conversions"
Cohesion: 0.21
Nodes (22): bno055_convert_double_gyro_x_dps(), bno055_convert_double_gyro_x_rps(), bno055_convert_double_gyro_xyz_dps(), bno055_convert_double_gyro_xyz_rps(), bno055_convert_double_gyro_y_dps(), bno055_convert_double_gyro_y_rps(), bno055_convert_double_gyro_z_dps(), bno055_convert_double_gyro_z_rps() (+14 more)

### Community 11 - "MPC Design Rationale"
Cohesion: 0.12
Nodes (13): Directional segment logic, Euler-discretized linear model, Feedforward release fallback, Fixed-size dense QP, Legacy units boundary, Serial Subscriber Map, CMovelistexecutor(), MPC Controller Design Notes (+5 more)

### Community 12 - "Temperature Handling"
Cohesion: 0.57
Nodes (7): bno055_convert_double_temp_celsius(), bno055_convert_double_temp_fahrenheit(), bno055_convert_float_temp_celsius(), bno055_convert_float_temp_fahrenheit(), bno055_get_temp_unit(), bno055_read_temp_data(), bno055_set_temp_unit()

### Community 13 - "Gyro Config"
Cohesion: 0.4
Nodes (6): bno055_get_gyro_auto_sleep_durn(), bno055_get_gyro_bw(), bno055_get_gyro_power_mode(), bno055_gyro_set_auto_sleep_durn(), bno055_set_gyro_bw(), bno055_set_gyro_power_mode()

### Community 14 - "Battery Manager"
Cohesion: 0.5
Nodes (1): CBatterymanager()

### Community 15 - "Blinker Task"
Cohesion: 0.5
Nodes (1): CBlinker()

### Community 16 - "Component Generator"
Cohesion: 0.67
Nodes (0): 

### Community 17 - "Linear Accel X"
Cohesion: 0.67
Nodes (3): bno055_convert_double_linear_accel_x_msq(), bno055_convert_float_linear_accel_x_msq(), bno055_read_linear_accel_x()

### Community 18 - "Magnetometer X"
Cohesion: 0.67
Nodes (3): bno055_convert_double_mag_x_uT(), bno055_convert_float_mag_x_uT(), bno055_read_mag_x()

### Community 19 - "Gravity Y"
Cohesion: 0.67
Nodes (3): bno055_convert_gravity_double_y_msq(), bno055_convert_gravity_float_y_msq(), bno055_read_gravity_y()

### Community 20 - "Gravity XYZ"
Cohesion: 0.67
Nodes (3): bno055_convert_double_gravity_xyz_msq(), bno055_convert_float_gravity_xyz_msq(), bno055_read_gravity_xyz()

### Community 21 - "Gravity X"
Cohesion: 0.67
Nodes (3): bno055_convert_gravity_double_x_msq(), bno055_convert_gravity_float_x_msq(), bno055_read_gravity_x()

### Community 22 - "Linear Accel Z"
Cohesion: 0.67
Nodes (3): bno055_convert_double_linear_accel_z_msq(), bno055_convert_float_linear_accel_z_msq(), bno055_read_linear_accel_z()

### Community 23 - "Magnetometer Y"
Cohesion: 0.67
Nodes (3): bno055_convert_double_mag_y_uT(), bno055_convert_float_mag_y_uT(), bno055_read_mag_y()

### Community 24 - "Build & Compile"
Cohesion: 1.0
Nodes (3): Mbed OS cross-compilation setup, robot_car application target, mbed-tools compile command

### Community 25 - "Queue API"
Cohesion: 1.0
Nodes (1): CQueue

### Community 26 - "Project Overview"
Cohesion: 1.0
Nodes (2): BFMC embedded platform project, Communication protocol between RPi and Nucleo

### Community 27 - "Main Header"
Cohesion: 1.0
Nodes (0): 

### Community 28 - "Global State Decls"
Cohesion: 1.0
Nodes (0): 

### Community 29 - "BNO055 Header"
Cohesion: 1.0
Nodes (0): 

### Community 30 - "Global State Storage"
Cohesion: 1.0
Nodes (0): 

### Community 31 - "Queue Implementation"
Cohesion: 1.0
Nodes (0): 

## Knowledge Gaps
- **17 isolated node(s):** `CEncoder`, `ISpeedingCommand`, `ISteeringCommand`, `CQueue`, `Fixed-size dense QP` (+12 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Queue API`** (2 nodes): `queue.hpp`, `CQueue`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Project Overview`** (2 nodes): `BFMC embedded platform project`, `Communication protocol between RPi and Nucleo`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Main Header`** (1 nodes): `main.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Global State Decls`** (1 nodes): `globalsv.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `BNO055 Header`** (1 nodes): `BNO055.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Global State Storage`** (1 nodes): `globalsv.cpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Queue Implementation`** (1 nodes): `queue.cpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `_run()` connect `Euler Angle Conversions` to `Periodic Task Headers`, `Move Executor Logic`, `Test Command Hooks`?**
  _High betweenness centrality (0.369) - this node is a cross-community bridge._
- **Why does `getLinearSpeed()` connect `Move Executor Logic` to `Periodic Task Headers`, `Euler Angle Conversions`, `Encoder Filtering`?**
  _High betweenness centrality (0.238) - this node is a cross-community bridge._
- **Why does `applyInterpolatedCommand()` connect `Move Executor Logic` to `Test Command Hooks`, `MPC Solver Assembly`?**
  _High betweenness centrality (0.176) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `applyInterpolatedCommand()` (e.g. with `solve()` and `getLinearSpeed()`) actually correct?**
  _`applyInterpolatedCommand()` has 6 INFERRED edges - model-reasoned connections that need verification._
- **What connects `CEncoder`, `ISpeedingCommand`, `ISteeringCommand` to the rest of the system?**
  _17 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Linear/Mag Conversions` be split into smaller, more focused modules?**
  _Cohesion score 0.03 - nodes in this community are weakly interconnected._
- **Should `BNO055 Motion Config` be split into smaller, more focused modules?**
  _Cohesion score 0.08 - nodes in this community are weakly interconnected._