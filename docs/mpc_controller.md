# MPC Controller

## Overview
- `CMpcController` is a self-contained, fixed-size Level 2 linear MPC used by `CMovelistexecutor` as a correction layer around the planner feedforward.
- The planner remains unchanged and still provides nominal pose, speed, steer, and segment direction.
- The executor applies commands as `v_cmd = v_ref + v_corr` and `delta_cmd = delta_ff + delta_corr`.

## State And Inputs
- State: `x = [e_s, e_y, e_psi]`
- Input: `u = [v_corr, delta_corr]`
- `e_s` is matched-progress minus nominal time-progress, in meters.
- `e_y` is lateral path-frame error, in meters.
- `e_psi` is body heading error relative to the path-frame heading `theta_ref`, in radians.
- `v_corr` is a path-speed correction in m/s.
- `delta_corr` is a steering correction in rad.

## Units
- Internal MPC units are SI only: `m`, `m/s`, `rad`.
- Executor keyframes and actuator interfaces stay in legacy units: `mm`, `mm/s`, `deci-deg`.
- Unit conversion happens at the executor boundary before the controller call and after command reconstruction.

## Segment Logic
- The executor precomputes contiguous fixed-direction segments from uploaded keyframes.
- Dynamic forward and reverse segments use the MPC.
- Zero-speed dwell segments bypass the MPC and command zero speed and zero steer.
- Entering a new segment resets the MPC warm start and previous correction history.
- Reverse segments use `theta_ref = wrap(psi_ref + pi)` for path-frame lateral error computation.
- Reverse segments use `e_psi = wrap(psi - theta_ref)` so lateral and heading error stay in the same path frame.

## Horizon And Model
- Sample time: `Ts = 0.05 s`
- Horizon length: `N = 10`
- Prediction horizon: `0.5 s`
- Stage 0 uses the current matched-progress planner feedforward.
- Future stages are sampled from the nominal time schedule and clamped to the active segment time window.
- The controller uses the Euler-discretized linear model around the current segment direction, `v_ref = abs(v_ref_body)`, and `kappa_ref = -dir * tan(delta_ff) / L`.
- The negative sign on `kappa_ref` matches the car's steering convention, where positive steering commands turn the wheels to the right.

## Tuning
- State normalization:
  - `e_s_scale = 0.05 m`
  - `e_y_scale = 0.02 m`
  - `e_psi_scale = 0.0872665 rad`
- Input normalization:
  - `v_corr_scale = 0.05 m/s`
  - `delta_corr_scale = 0.0698132 rad`
- Weights:
  - `Q = diag([1.0, 6.0, 4.0])`
  - `R = diag([0.35, 0.80])`
  - `S = diag([0.25, 1.40])`
  - `P = diag([2.0, 12.0, 8.0])`

## Constraints
- Path-speed magnitude bounds are enforced per stage with fixed segment direction.
- Steering bounds are enforced around `delta_ff`.
- Speed and steering slew-rate limits are enforced against the previous applied command at stage 0 and against previous horizon stages afterward.
- No corridor constraints or direction optimization are used in this version.

## Solver And Fallback
- The controller builds a condensed dense QP with fixed stack sizes and no heap allocation.
- The QP is solved with projected gradient descent plus cyclic half-space projection.
- Warm start shifts the previous horizon solution forward by one stage.
- On large disturbances, warm start and previous-correction memory are cleared before solving.
- If projected gradient reaches the iteration cap but the candidate solution is still finite and only has a small residual constraint violation, the controller applies that iterate instead of discarding it.
- If the QP still fails numerically, execution continues with a rate-limited release toward the current feedforward command rather than latching an old correction.
- The executor still applies startup minimum-speed assist and final actuator clamps after MPC command reconstruction.
- Progress matching is stabilized before the MPC call so the executor does not jump backward or forward into unrelated planner points on tight curves or near dwell transitions.
