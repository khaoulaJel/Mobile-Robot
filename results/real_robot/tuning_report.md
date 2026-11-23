# Heading control on the professor's robot: parameters used, what went wrong, what was improved and tuned

**Result type: simulation.** The robot is the professor's Simulink model (deadband 0.2 rad/s, slow identified velocity response); the pose source is
modelled after what the real network showed. Nothing here has been measured on the real robot yet.

## 1. The parameters of the run (extracted)

`results/real_robot/params_used_run1.yaml` is the complete effective set of the run that ended about 7 degrees off (file `reference_lab_robot.yaml` + the
command-line topics + defaults). The heading-relevant part:

| Parameter | Value | Meaning |
| --- | --- | --- |
| `kp` | 1.0 | proportional gain, `w = kp * error` |
| `tolerance` | 0.01 rad | accepted final error |
| `max_angular_speed` | 1.0 rad/s | |
| `deadband_w`, `deadband_v` | 0.2 rad/s, 0.04 m/s | the robot ignores smaller commands |
| `lead_w` | 0.8 s | predicted rest heading = heading + turn rate x 0.8 s |
| `vel_window`, `vel_filter` | 5 samples, 0.7 | speed estimate from finite differences of the pose |
| `rest_window`, `rest_yaw`, `rest_timeout`, `settle_time`, `rest_retries` | 1.0 s, 0.01 rad, 4 s, 3 s, 4 | wait for rest, check the true error, correct with pulses |
| state topic | `/robot_pose` (map frame) | slow, late, noisy, freezing |

## 2. Why it was off

1. **The frame.** The first runs used the wheel odometry; the professor measures in the map frame (`/robot_pose`), which was rotated by about 77 degrees.
2. **The pose source.** The map pose arrives slowly (3-25 Hz), 0.05-0.6 s late, with noise, and the data freezes for about 3 s every 35 s (`state is stale`).
3. **The controller assumed a good state.** It reused the same pose up to six times per sample (the 20 Hz loop), so the speed estimate flickered; it extrapolated
   that noisy speed over about a second of lead; and it kept its estimates through a freeze.

In the simulator with these conditions the plain P law ends **7.0 degrees** off on average (the observed error), the preset about 0.8 degrees, with overshoot of 11 degrees.
Overshoot correlates with the delay (+0.65) and the robot's gain (+0.41).

## 3. What was changed in the controller

| Change | Where | Why |
| --- | --- | --- |
| **Timed mode**: the controller receives each sample's time stamp and age; the speed estimate uses only NEW samples, with the real time between them | `compensation.py` (`_predict_timed`) | the 20 Hz loop reused each pose; the speed estimate flickered |
| Speed = **least-squares slope** over `vel_span` seconds, clamped to what the robot can do | `compensation.py` (`_slopes`) | noise amplified by the long prediction horizon made the naive timed mode oscillate (worst case 76 deg) |
| Estimates are **cleared after a gap** longer than 1 s | `compensation.py` | stale numbers must not feed the first samples after a freeze |
| Rest test by **window averages** (`rest_mean_test`), window in seconds | `compensation.py` | single noisy samples never look "at rest" |
| **Model-based observer**: simulates the robot (deadband, lag, gain) from the commands we really sent and uses the late pose only to correct it | `observer.py`, `tracker.py` | the prediction no longer depends on noisy differences; it keeps running through a freeze |
| **PID form**: integral term near the target (`ki`, anti-windup) | `tracker.py` | pushes through the last degrees |
| The node tells the model what it really sent, including the stop on stale data | `reference_controller.py` | the model must know the robot was stopped |

All of it is **off by default**: with the new options off the controller is byte-identical to the previous one (9,797 steps compared, the 14-shape benchmark hash).

## 4. Tuning and results

`tools/tune_heading_real.py`: random search then local refinement of the coefficients on 60 training cases, then judged on **160 validation cases the search never saw**
(80 normal, 80 stress: slow source, long freezes, robot gain +-20 %, SLAM jumps of 2-6 degrees).

| Controller | Normal: mean error | within 1 deg | overshoot | STRESS: mean error | within 1 deg | overshoot | worst |
| --- | --- | --- | --- | --- | --- | --- | --- |
| plain P law | 4.71 deg | 12 % | 24.6 deg | 8.11 deg | 4 % | 36.9 deg | 22.9 deg |
| preset used in the run | 0.47 | 92 % | 3.2 | 0.84 | 69 % | 12.5 | 4.4 |
| naive timed mode, tuned | 0.47 | 90 % | 0.4 | 1.10 | 71 % | 4.3 | 11.0 |
| classic, tuned | 0.32 | 98 % | 0.7 | 0.56 | 90 % | 5.8 | 3.6 |
| **observer + PID, tuned** | **0.31** | **100 %** | **0.03** | **0.51** | **90 %** | **3.0** | **3.0** |

The tuned set is `config/reference_lab_robot_tuned.yaml` (`kp 0.98`, `ki 0.044`, `max_angular_speed 0.70`, `obs_tau 0.62`, `obs_gain 0.93`, `pose_latency 0.54`, ...).
Settling takes about 10 s (normal) to 19 s (stress): the controller deliberately waits for rest before the last corrections.

## 5. Limits and what the real robot has to confirm

- The robot model is the professor's, not measured. If the real deadband, lag or gain differ, the observer is less accurate: measure with `tools/measure_response.py`
  and re-tune with `tools/tune_heading_real.py` (edit `FIXED` and the search ranges).
- Path tracking (pose references) is not tuned here; it runs in the classic mode.
- The freezes limit the last degree no matter the controller: a controller cannot correct what it cannot measure. A wired network would remove them.
- To re-tune on real data, record per trial: the commands sent, the `/robot_pose` heading with its time stamps, and the target (see `docs/real_robot.md`).

## 6. Reproduce

```bash
python3 tools/yaw_real_sim.py --cases 60                      # the conditions and the comparison
python3 tools/tune_heading_real.py --modes classic,observer   # the search (a few minutes with 12 workers)
python3 tools/effective_params.py config/reference_lab_robot.yaml --set state_topic=/robot_pose   # the parameters of a run
PYTHONPATH=. python3 -m pytest -q test                         # 55 unit tests, including test_observer.py
```
