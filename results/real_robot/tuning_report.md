# The P family on the professor's robot: parameters used, what went wrong, the study, the defended design

**Result type: simulation.** The robot is the professor's Simulink model (command deadband, slow identified response); the pose source is modelled after what the real network
showed (slow, late, noisy, freezing). Nothing here has been measured on the real robot yet.

## 1. The parameters of the run that ended about 7 degrees off

`results/real_robot/params_used_run1.yaml` (extracted with `tools/effective_params.py`): `kp 1.0`, `tolerance 0.01`, `max_angular_speed 1.0`, `lead_w 0.8`,
`vel_window 5`, `vel_filter 0.7`, `rest_window 1.0`, deadband 0.2 rad/s, state `/robot_pose`.

## 2. Why it was off

1. **Frame**: the first runs used the wheel odometry, the professor measures in the map frame (`/robot_pose`, about 77 degrees rotated).
2. **Pose source**: the map pose arrives at 3-25 Hz, 0.05-0.6 s late, noisy, with freezes of about 3 s every 35 s.
3. **Structure**: a bare P law cannot beat the robot's deadband (small commands are dropped), and the old code reused the same pose up to 6 times per sample and
   extrapolated a noisy speed over about a second.
   In the simulator the plain P law ends 7 degrees off on average, which is what was observed.

## 3. What was added to the implementation (all opt-in; with the options off the controller is byte-identical to the previous one)

| Change | Where |
| --- | --- |
| Timed mode: stamps and age of each sample, speed from a least-squares slope of NEW samples, estimates cleared after a gap, rest test by window averages | `compensation.py` |
| Model-based observer: simulates the robot (deadband, lag, gain) from the commands really sent, corrected by the late pose; for the heading and for a forward position | `observer.py` |
| P / PI / PD / PID forms for the heading (`YawTracker`) and a new forward-position controller (`LinearTracker`, x and x-dot) | `tracker.py` |
| The node feeds the controller the stamps, the age and what it really sent, including the stop on stale data | `reference_controller.py` |
| Real-conditions simulators and the study tools | `tools/yaw_real_sim.py`, `x_real_sim.py`, `p_family_study.py`, `x_family_study.py`, `p_family_robustness.py` |

## 4. The study (fair comparison)

Five structures, **each tuned with the same optimiser, the same budget (240 random + 160 refinement evaluations) and the same training cases** (30 normal + 30 stress + 30 wrong-robot);
only its own coefficients are free. Then judged on **300 validation cases none of them was tuned on** (100 per condition set). Mean final error +- standard error.

* normal: moderate delay, noise and freezes; stress: slow source (3-15 Hz), delay up to 0.6 s, noise up to 1.5 deg / 2 cm, freezes of 2-6 s, robot gain +-20 %, SLAM jumps;
* **wrong robot**: stress, plus the REAL robot differs from the model the controller knows (deadband 0.12-0.30 rad/s while it is told 0.2; extra lag 0-0.4 s).

### Heading (degrees)

| Structure | Normal | Stress | Wrong robot | within 1 deg (wrong robot) | worst (wrong robot) |
| --- | --- | --- | --- | --- | --- |
| P | 4.56 +- 0.28 | 5.86 +- 0.42 | 8.40 +- 0.54 | 7 % | 29.6 |
| PI | 4.65 +- 0.26 | 5.62 +- 0.36 | 5.95 +- 0.47 | 12 % | 17.1 |
| **PD** (observer + lead) | **0.26 +- 0.02** | **0.36 +- 0.04** | **2.31 +- 0.31** | 55 % | 16.0 |
| PID | 0.17 +- 0.02 | 0.27 +- 0.03 | 3.14 +- 0.43 | 55 % | 15.7 |
| PID + rest/pulse layer | 0.41 +- 0.04 | 0.59 +- 0.05 | 2.23 +- 0.30 | 56 % | 12.8 |

**Decision: PD.** P and PI cannot beat the deadband (4-8 degrees). The derivative, fed by the observer's turn rate, is what makes the difference. PD beats the rest/pulse layer
on normal and stress cases and ties it with a wrong robot; PID is 0.09 degrees better when the model is right (significant) but 0.8 degrees worse when it is wrong (within noise), and the real
robot's model is exactly what is unknown. PD is also the simplest of the three. (An earlier version of this work shipped PID + layer as the preset; this study showed it is not the best.)
Preset: `config/reference_lab_robot_tuned.yaml` (`kp 1.28`, `lead_w 0.77`, `max_angular_speed 0.60`, observer lag 0.54 s, gain 0.92, pose delay 0.62 s).

### Forward position x and x-dot (centimetres)

| Structure | Normal | Stress | Wrong robot | within 1 cm (wrong robot) | worst (wrong robot) |
| --- | --- | --- | --- | --- | --- |
| P | 0.46 +- 0.04 | 0.82 +- 0.10 | 3.95 +- 1.29 | 53 % | 73.1 |
| PI | 0.49 +- 0.04 | 0.64 +- 0.08 | 1.01 +- 0.11 | 67 % | 5.9 |
| PD | 0.19 +- 0.02 | 0.26 +- 0.02 | 1.11 +- 0.15 | 70 % | 6.9 |
| **PID** (observer) | **0.15 +- 0.01** | **0.29 +- 0.03** | **0.58 +- 0.06** | **82 %** | **2.7** |
| PID + rest/pulse layer | 0.18 +- 0.02 | 0.38 +- 0.03 | 1.19 +- 0.14 | 64 % | 5.9 |

**Decision: PID** for the position: best or tied on every row, and 0.5-0.9 cm in every robustness test. The integral term is what pushes through the forward-speed deadband.
Tuned set (`LinearTracker`, `results/real_robot/x_family_study.json`): `kv 1.47`, `ki_v 0.29`, `lead_v 1.11`, `max_linear_speed 0.18`, `pos_tolerance 0.0084`,
observer lag 0.22 s, gain 1.30, pose delay 0.15 s.

### Robustness (heading, mean error in degrees, tuned coefficients unchanged, robot different from what they were tuned for)

| Condition | P | PI | PD | PID | PID+layer |
| --- | --- | --- | --- | --- | --- |
| robot gain x0.7 | 6.98 | 4.85 | 3.77 | 5.53 | 3.43 |
| robot gain x1.3 | 12.44 | 9.74 | 2.85 | 3.76 | 4.20 |
| extra pose delay +0.3 s | 10.37 | 8.33 | 2.99 | 3.54 | 3.57 |
| pose noise x2 | 9.04 | 5.83 | 2.59 | 4.07 | 2.61 |
| freezes x2 longer | 9.81 | 6.68 | 2.67 | 4.18 | 2.65 |
| pose rate x0.5 | 9.79 | 7.37 | 2.72 | 4.49 | 2.48 |
| true deadband 0.12 (model says 0.2) | 9.21 | 9.56 | 0.62 | 0.90 | 0.64 |
| **true deadband 0.30 (model says 0.2)** | 9.59 | 4.94 | **5.54** | 7.85 | 5.22 |
| robot lag +0.4 s (model unaware) | 9.79 | 7.39 | 2.54 | 4.31 | 2.95 |
| beyond the tuning range (2.5 Hz, +0.5 s, noise x2.5, 8 s freezes) | 17.45 | 14.26 | 4.39 | 6.45 | 5.36 |

(An earlier tuning WITHOUT wrong-robot cases in training gave PD 0.11 degrees nominally but **46 degrees** with a real deadband of 0.30: it trusted its model too much.
Putting model mismatch into the training is what fixed that, at the price of a slightly worse nominal error.)

## 5. A negative result: adaptive deadband

The controller can notice that the robot ignores small commands (the measured rate stays far below the command for 2 s) and raise its believed deadband.
Tested on the tuned structures without retuning: heading PD with a true deadband of 0.30 improves from 7.3 to 5.0 degrees (worst case 16 to 9.3), but PID gains almost nothing,
PID+layer gets worse in stress (0.59 to 0.98) and the position PID+layer gets a worst case of 34 cm. **Mixed results, so it stays OFF** (`adapt_deadband: false`) and is kept as an option.

## 6. Limits

* The remaining error is dominated by one unknown: **the robot's true deadband**. A real deadband well above the 0.2 rad/s of the model gives about 5 degrees. Measure it
  (`tools/measure_response.py`), put it in `deadband_w` / `deadband_v`, and re-tune (`tools/p_family_study.py`, `tools/x_family_study.py`).
* The freezes of the pose limit the last degree whatever the controller (it cannot correct what it cannot measure). A wired network removes them.
* Position control exists as a controller (`LinearTracker`) with tests; its use inside the node (a `/reference_x` topic, and a combined go-to-point controller made of the heading PD
  and the position PID) is the next step. Pose references in the node still run in the classic mode.
* All numbers are simulation results with the professor's robot model.

## 7. Reproduce

```bash
python3 tools/yaw_real_sim.py --cases 60                          # the heading conditions, plain P vs preset
python3 tools/p_family_study.py --workers 12                      # heading: the five structures (about 30 minutes)
python3 tools/x_family_study.py --workers 12                      # position: the five structures
python3 tools/p_family_robustness.py results/real_robot/p_family_study.json     # robustness table
PYTHONPATH=. python3 -m pytest -q test                            # 76 tests, including test_tuned_heading_preset.py (the preset keeps its claimed accuracy)
```
