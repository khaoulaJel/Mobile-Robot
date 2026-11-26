#!/usr/bin/env bash
# Strict acceptance suite for lab3_controllers (simulation/software only; hardware is separate).
# Usage (WSL/Ubuntu 22.04 with ROS 2 Humble + Gazebo Classic):
#     bash run_all.sh [repo_dir]             # the repository root (contains src/ and results/); default: this repository
#   src/ must also contain the Gazebo workspace packages cc_robot_urdf and lab3_gazebo (course material, see README).
# Exit code 0 only if EVERY test passed. Each test prints PASS/FAIL; a final table lists failures.
set -uo pipefail

LAB="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
WS="$HOME/lab3_acceptance"
OUT="$WS/out"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-86}"
PASS=0; FAIL=0; FAILED=()
PIDS=()
src() { set +u; source "$1"; set -u; }   # ROS setup scripts use unset variables

start_bg() {            # start_bg LOGFILE cmd args...  -> sets LAST_PID (own process group)
    local log="$1"; shift
    setsid "$@" > "$log" 2>&1 &
    LAST_PID=$!
    PIDS+=("$LAST_PID")
}
stop_bg() {             # stop_bg PID : SIGINT the group, then SIGKILL if it lingers
    local pid="$1"
    kill -INT -- "-$pid" 2>/dev/null || return 0
    for _ in $(seq 50); do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.1; done
    kill -KILL -- "-$pid" 2>/dev/null || true
}
cleanup() {
    for p in "${PIDS[@]:-}"; do [ -n "$p" ] && stop_bg "$p"; done
    pkill -f "[g]zserver" 2>/dev/null; pkill -f "[g]zclient" 2>/dev/null
    true
}
trap cleanup EXIT

run_test() {
    local name="$1"; shift
    echo "===== $name ====="
    if "$@"; then echo "PASS: $name"; PASS=$((PASS + 1))
    else echo "FAIL: $name"; FAIL=$((FAIL + 1)); FAILED+=("$name"); fi
}
wait_for() {            # wait_for FILE PATTERN SECONDS
    for _ in $(seq $(( $3 * 2 ))); do grep -q -- "$2" "$1" 2>/dev/null && return 0; sleep 0.5; done
    return 1
}

# ---------------------------------------------------------------- tests
t_unit() {
    (cd "$LAB/src/lab3_controllers" && PYTHONPATH=. python3 -m pytest -q test -p no:cacheprovider)
}
t_build() {
    for pkg in cc_robot_urdf lab3_gazebo; do
        [ -d "$LAB/src/$pkg" ] || { echo "missing src/$pkg: copy the Gazebo workspace packages from the course lab3.zip into src/"; return 1; }
    done
    rm -rf "$WS" && mkdir -p "$WS/src" "$OUT" && cp -r "$LAB/src/." "$WS/src/" || return 1
    # shellcheck disable=SC1091
    src /opt/ros/humble/setup.bash
    (cd "$WS" && colcon build --symlink-install) || return 1
    [ -f "$WS/install/lab3_controllers/lib/lab3_controllers/motion_controller" ] || { echo "motion_controller missing"; return 1; }
    [ -f "$WS/install/lab3_controllers/lib/lab3_controllers/yaw_controller" ]    || { echo "yaw_controller missing"; return 1; }
}
t_safety() { timeout 420 python3 -u "$WS/src/lab3_controllers/tools/test_safety.py"; }
t_p_controller() { timeout 560 python3 -u "$WS/src/lab3_controllers/tools/check_p_controller.py"; }     # the real-robot P controller node
t_reference() { timeout 400 python3 -u "$WS/src/lab3_controllers/tools/check_reference.py"; }     # reference from a topic, live coefficients
t_gazebo_up() {
    start_bg "$OUT/gazebo.log" ros2 launch lab3_gazebo gazebo.launch.py gui:=false
    GZ_PID=$LAST_PID
    for _ in $(seq 40); do
        timeout 10 ros2 topic echo /odom --once > "$OUT/odom_once.txt" 2>/dev/null && grep -q "pose" "$OUT/odom_once.txt" && break
        sleep 1
    done
    grep -q "pose" "$OUT/odom_once.txt" || return 1
    timeout 10 ros2 topic info /diff_cont/cmd_vel_unstamped > "$OUT/cmd_info.txt" 2>&1
    grep -q "Type: geometry_msgs/msg/Twist" "$OUT/cmd_info.txt"
}
t_yaw_launch() {
    start_bg "$OUT/yaw_launch.log" ros2 launch lab3_controllers yaw.launch.py use_sim_time:=true
    local pid=$LAST_PID
    wait_for "$OUT/yaw_launch.log" "yaw reached" 30; local rc=$?
    stop_bg "$pid"; sleep 1
    return $rc
}
t_yaw_validate() {
    start_bg "$OUT/yaw_node.log" ros2 run lab3_controllers yaw_controller --ros-args -p use_sim_time:=true
    local pid=$LAST_PID; sleep 2
    timeout 120 python3 -u "$WS/src/lab3_controllers/tools/validate_yaw.py" "$OUT/yaw.csv" | tee "$OUT/yaw_validate.txt"
    local rc=${PIPESTATUS[0]}
    stop_bg "$pid"; sleep 1
    ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{}' > /dev/null 2>&1
    [ "$rc" -eq 0 ] && [ -s "$OUT/yaw.csv" ]
}
t_motion_launch() {
    start_bg "$OUT/motion_launch.log" ros2 launch lab3_controllers motion.launch.py use_sim_time:=true
    local pid=$LAST_PID
    wait_for "$OUT/motion_launch.log" "path complete" 90; local rc=$?
    stop_bg "$pid"; sleep 1
    return $rc
}
t_run() {               # t_run SHAPE STRATEGY
    local shape="$1" strat="$2" csv="$OUT/${1}_${2}.csv" log="$OUT/${1}_${2}.log" extra=()
    [ "$shape" = circle ] && extra=(-p path_shape:=circle)
    start_bg "$log" ros2 run lab3_controllers motion_controller --ros-args -p use_sim_time:=true \
        -p strategy:="$strat" -p relative_to_start:=true -p log_csv:="$csv" "${extra[@]}"
    local pid=$LAST_PID
    wait_for "$csv" ",DONE," 300; local rc=$?
    sleep 1; stop_bg "$pid"; sleep 1
    [ "$rc" -eq 0 ] && grep -q "path complete" "$log"
}
t_repeat() {            # REPEATS identical stop_turn square runs: mean/std/worst error
    local n="${REPEATS:-10}" files=() k
    for k in $(seq "$n"); do
        local csv="$OUT/rep_$k.csv" log="$OUT/rep_$k.log"
        start_bg "$log" ros2 run lab3_controllers motion_controller --ros-args -p use_sim_time:=true             -p relative_to_start:=true -p log_csv:="$csv"
        local pid=$LAST_PID
        wait_for "$csv" ",DONE," 120 || { stop_bg "$pid"; echo "run $k did not finish"; return 1; }
        sleep 1; stop_bg "$pid"; sleep 1
        files+=("$csv")
    done
    python3 "$WS/src/lab3_controllers/tools/trial_stats.py" "${files[@]}" | tee "$OUT/repeatability.txt"
    python3 - "${files[@]}" <<'PY'
import os, statistics, sys
sys.path.insert(0, os.path.expanduser('~/lab3_acceptance/src/lab3_controllers/tools'))
import trial_stats as ts
files = sys.argv[1:]
res = [ts.one(f) for f in files]
pos = [r[1] for r in res]; yaw = [r[2] for r in res]
ok = max(pos) <= 1.5 and max(yaw) <= 1.5 and statistics.pstdev(pos) <= 0.3
print(f'{len(res)} runs: pos mean {statistics.mean(pos):.2f} std {statistics.pstdev(pos):.2f} max {max(pos):.2f} cm; '
      f'yaw max {max(yaw):.2f} deg  -> limits: max pos 1.5, max yaw 1.5, std pos 0.3')
sys.exit(0 if ok else 1)
PY
}
t_robust_preset() {
    start_bg "$OUT/robust.log" ros2 launch lab3_controllers motion.launch.py config:=robust.yaml use_sim_time:=true
    local pid=$LAST_PID
    wait_for "$OUT/robust.log" "path complete" 300; local rc=$?
    stop_bg "$pid"; sleep 1
    [ "$rc" -eq 0 ] && grep -q "strategy=stop_turn" "$OUT/robust.log"
}
t_lab_square() {        # lab_robot.yaml, motion controller, through the professor-robot relay (filter + slow plant)
    local csv="$OUT/lab_square.csv" T="$WS/src/lab3_controllers/tools"
    local cfg="$WS/install/lab3_controllers/share/lab3_controllers/config/lab_robot.yaml"
    start_bg "$OUT/lab_relay.log" python3 "$T/lab_robot_relay.py" --ros-args -p use_sim_time:=true
    local rpid=$LAST_PID; sleep 2
    start_bg "$OUT/lab_square.log" ros2 run lab3_controllers motion_controller --ros-args --params-file "$cfg"         -p use_sim_time:=true -p cmd_topic:=/lab/cmd_vel -p log_csv:="$csv"
    local pid=$LAST_PID
    wait_for "$csv" ",DONE," 400; local rc=$?
    if [ "$rc" -eq 0 ]; then
        python3 -u "$T/lab_robot_gazebo_check.py" "$csv" --shape square | tee "$OUT/lab_square.txt"; rc=${PIPESTATUS[0]}
    else
        echo "controller did not finish within 400 s"
    fi
    stop_bg "$pid"; stop_bg "$rpid"; sleep 1
    ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{}' > /dev/null 2>&1
    return $rc
}
t_lab_yaw() {           # lab_robot.yaml, yaw controller, five references, through the relay
    local T="$WS/src/lab3_controllers/tools"
    local cfg="$WS/install/lab3_controllers/share/lab3_controllers/config/lab_robot.yaml"
    start_bg "$OUT/lab_relay_yaw.log" python3 "$T/lab_robot_relay.py" --ros-args -p use_sim_time:=true
    local rpid=$LAST_PID; sleep 2
    start_bg "$OUT/lab_yaw_node.log" ros2 run lab3_controllers yaw_controller --ros-args --params-file "$cfg"         -p use_sim_time:=true -p cmd_topic:=/lab/cmd_vel
    local pid=$LAST_PID; sleep 2
    timeout 400 python3 -u "$T/validate_yaw.py" "$OUT/lab_yaw.csv" 40 | tee "$OUT/lab_yaw.txt"
    local rc=${PIPESTATUS[0]}
    stop_bg "$pid"; stop_bg "$rpid"; sleep 1
    ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{}' > /dev/null 2>&1
    [ "$rc" -eq 0 ] || return 1
    python3 - "$OUT/lab_yaw.txt" <<'PY'
import re, sys
errs = [float(m) for m in re.findall(r'error\s+(-?[0-9.]+) deg', open(sys.argv[1]).read())]
assert len(errs) == 5, f'expected 5 references, got {errs}'
print('final errors [deg]:', errs, '(limit 1.0)')
assert max(abs(e) for e in errs) <= 1.0, errs
PY
}
t_circle_p() {          # Task 2 of the lab note: P controller, circle of 1 m diameter, tangent start at (0, 0)
    local strat="$1" csv="$OUT/circle1m_${1}.csv" log="$OUT/circle1m_${1}.log" T="$WS/src/lab3_controllers/tools"
    start_bg "$log" ros2 run lab3_controllers motion_controller --ros-args -p use_sim_time:=true         -p strategy:="$strat" -p path_shape:=circle -p circle_radius:=0.5 -p relative_to_start:=true -p log_csv:="$csv"
    local pid=$LAST_PID
    wait_for "$csv" ",DONE," 400; local rc=$?
    sleep 1; stop_bg "$pid"; sleep 1
    [ "$rc" -eq 0 ] || { echo "did not finish"; return 1; }
    python3 "$T/check_circle.py" "$csv" --radius 0.5 --max-radial 0.015 --max-final 0.015 | tee "$OUT/circle1m_${strat}.txt"
    return ${PIPESTATUS[0]}
}
t_symbolic() {          # Task 3 of the lab note: symbolic (abstraction-based) controller, same circle, Gazebo
    local csv="$OUT/symbolic_circle.csv" log="$OUT/symbolic_circle.log" T="$WS/src/lab3_controllers/tools"
    start_bg "$log" ros2 run lab3_controllers symbolic_controller --ros-args -p use_sim_time:=true -p log_csv:="$csv"
    local pid=$LAST_PID
    wait_for "$log" "lap complete\|stopping" 900; local rc=$?
    sleep 2; stop_bg "$pid"; sleep 1
    ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{}' > /dev/null 2>&1
    [ "$rc" -eq 0 ] || { echo "did not finish"; return 1; }
    grep -E "lap complete|fallback|stopping" "$log" | tail -3
    python3 "$T/check_circle.py" "$csv" --radius 0.5 --max-radial 0.08 --max-final 0.08 | tee "$OUT/symbolic_circle.txt"
    return ${PIPESTATUS[0]}
}
t_measure() {           # response-measurement tool: Gazebo alone, then Gazebo behind the emulated professor robot
    local T="$WS/src/lab3_controllers/tools" alone="$OUT/measure_gazebo_alone.json" json="$OUT/measure_response.json"
    # 1. Gazebo alone: the drive plugin is not instant (measured ~0.4 s / 0.3 s coasting) and has no deadband
    timeout 400 python3 -u "$T/measure_response.py" --state-topic /odom --cmd-topic /diff_cont/cmd_vel_unstamped --out "$alone" | tee "$OUT/measure_gazebo_alone.txt"
    [ "${PIPESTATUS[0]}" -eq 0 ] || return 1
    # 2. behind the relay (the professor's plant + filter in front of Gazebo)
    start_bg "$OUT/measure_relay.log" python3 "$T/lab_robot_relay.py" --ros-args -p use_sim_time:=true
    local rpid=$LAST_PID; sleep 2
    timeout 400 python3 -u "$T/measure_response.py" --state-topic /odom --cmd-topic /lab/cmd_vel --out "$json" | tee "$OUT/measure_response.txt"
    local rc=${PIPESTATUS[0]}
    stop_bg "$rpid"; sleep 1
    ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist '{}' > /dev/null 2>&1
    [ "$rc" -eq 0 ] || return 1
    python3 - "$alone" "$json" "$T" <<'PY'
import json, sys
sys.path.insert(0, sys.argv[3])
import measure_response as mr
import plant_models as pm
g = json.load(open(sys.argv[1]))['result']
r = json.load(open(sys.argv[2]))['result']
# the emulated plant alone, measured with the SAME tool and analysis
sched = mr.build_schedule()
pv = mr.analyse(*mr.simulate(pm.IdentifiedPlant(use_filter=True), sched), sched)
for k in ('v', 'w'):
    expect = g[k]['coast_lag'] + pv[k]['coast_lag']      # coasting times of cascaded systems add
    got = r[k]['coast_lag']
    print(f"{k}: Gazebo alone {g[k]['coast_lag']:.2f} s + emulated plant {pv[k]['coast_lag']:.2f} s = {expect:.2f} s; measured behind the relay {got:.2f} s")
    assert abs(got - expect) <= 0.3, f'{k}: coasting {got:.2f} s differs from Gazebo + plant {expect:.2f} s by more than 0.3 s'
assert g['v_deadband'] == 0.0 and g['w_deadband'] == 0.0, 'Gazebo alone has no deadband: the tool reported one'
assert abs(r['v_deadband'] - 0.04) < 1e-9, f"v deadband {r['v_deadband']} (emulated filter drops |v| <= 0.04)"
assert abs(r['w_deadband'] - 0.20) < 1e-9, f"w deadband {r['w_deadband']} (emulated filter drops |w| <= 0.2)"
print('deadband: none for Gazebo alone; 0.04 / 0.20 behind the relay (as emulated)')
PY
}
t_launch_params() {     # which topics does each launch file really give the node? (config file vs command-line override)
    local fail=0 spec args node cmd state got_cmd got_state pid i
    local specs=(
        "motion.launch.py config:=hardware.yaml|motion_controller|/cmd_vel_tracker|/diff_cont/odom"
        "motion.launch.py|motion_controller|/diff_cont/cmd_vel_unstamped|/odom"
        "motion.launch.py config:=hardware.yaml cmd_topic:=/x state_topic:=/y|motion_controller|/x|/y"
        "yaw.launch.py config:=hardware.yaml|yaw_controller|/cmd_vel_tracker|/diff_cont/odom"
        "yaw.launch.py|yaw_controller|/diff_cont/cmd_vel_unstamped|/odom"
        "symbolic.launch.py|symbolic_controller|/diff_cont/cmd_vel_unstamped|/odom"
        "symbolic.launch.py cmd_topic:=/cmd_vel_tracker state_topic:=/diff_cont/odom|symbolic_controller|/cmd_vel_tracker|/diff_cont/odom"
        "reference.launch.py|reference_controller|/cmd_vel_tracker|/diff_cont/odom"
        "reference.launch.py cmd_topic:=/x state_topic:=/y|reference_controller|/x|/y")
    for spec in "${specs[@]}"; do
        IFS='|' read -r args node cmd state <<< "$spec"
        start_bg "$OUT/launch_params.log" ros2 launch lab3_controllers $args use_sim_time:=false
        pid=$LAST_PID; got_cmd=""; got_state=""
        for i in $(seq 40); do
            got_cmd=$(timeout 5 ros2 param get "/$node" cmd_topic 2>/dev/null | sed -n 's/.*: //p')
            [ -n "$got_cmd" ] && { got_state=$(timeout 5 ros2 param get "/$node" state_topic 2>/dev/null | sed -n 's/.*: //p'); break; }
            sleep 0.5
        done
        stop_bg "$pid"; sleep 1
        if [ "$got_cmd" = "$cmd" ] && [ "$got_state" = "$state" ]; then
            echo "ok   $args -> cmd $got_cmd, state $got_state"
        else
            echo "FAIL $args -> cmd '$got_cmd' (expected $cmd), state '$got_state' (expected $state)"; fail=1
        fi
    done
    return $fail
}
t_check()    { python3 "$WS/src/lab3_controllers/tools/check_results.py" "$OUT/yaw.csv" "$OUT"; }
t_evaluate() { python3 "$WS/src/lab3_controllers/tools/evaluate.py" "$OUT" > "$OUT/metrics_stdout.txt" && [ -s "$OUT/square_comparison.png" ] && [ -s "$OUT/circle_comparison.png" ]; }
t_calib() {
    local T="$WS/src/lab3_controllers/tools" s r
    s=$(timeout 90 python3 -u "$T/calibrate_odometry.py" straight --distance 1.0 --speed 0.1 --sim-time) || { echo "$s"; return 1; }
    echo "$s"
    r=$(timeout 90 python3 -u "$T/calibrate_odometry.py" rotate --angle 360 --cw --sim-time) || { echo "$r"; return 1; }
    echo "$r"
    local d a
    d=$(echo "$s" | sed -n 's/.*distance travelled: \([0-9.]*\) m.*/\1/p')
    a=$(echo "$r" | sed -n 's/.*ODOM rotation: \([0-9.]*\) deg.*/\1/p')
    [ -n "$d" ] && [ -n "$a" ] || { echo "could not parse output"; return 1; }
    python3 - "$d" "$a" <<'PY' || return 1
import sys
d, a = float(sys.argv[1]), float(sys.argv[2])
assert 0.99 <= d <= 1.02, f"straight: {d} m for 1.0 m commanded"
assert 359.0 <= a <= 361.5, f"rotate: {a} deg for 360 commanded"
PY
    python3 "$T/calibrate_odometry.py" calc-radius --odom 1.00 --real 0.96 | grep -q "multiplier = 0.9600" || return 1
    python3 "$T/calibrate_odometry.py" calc-separation --odom 365 --real 360 | grep -q "multiplier = 1.0139"
}
t_state_source() {      # inspector on Gazebo's two state topics (Odometry and PoseStamped)
    local T="$WS/src/lab3_controllers/tools"
    timeout 60 python3 -u "$T/check_state_source.py" --topic /odom --seconds 5 | tee "$OUT/state_odom.txt"
    [ "${PIPESTATUS[0]}" -eq 0 ] || return 1
    timeout 90 python3 -u "$T/check_state_source.py" --topic /robot_pose --seconds 5 --rotate --forward | tee "$OUT/state_robot_pose.txt"
    [ "${PIPESTATUS[0]}" -eq 0 ] && grep -q "PoseStamped" "$OUT/state_robot_pose.txt" && ! grep -q "^FAIL" "$OUT/state_robot_pose.txt"
}
t_pose_topic_run() {    # full square driven from a PoseStamped state topic (/robot_pose)
    local csv="$OUT/square_robot_pose.csv" log="$OUT/square_robot_pose.log"
    start_bg "$log" ros2 run lab3_controllers motion_controller --ros-args -p use_sim_time:=true         -p state_topic:=/robot_pose -p relative_to_start:=true -p log_csv:="$csv"
    local pid=$LAST_PID
    wait_for "$csv" ",DONE," 150; local rc=$?
    sleep 1; stop_bg "$pid"; sleep 1
    [ "$rc" -eq 0 ] && grep -q "PoseStamped" "$log" && python3 - "$csv" <<'PY'
import csv, math, sys
r = list(csv.DictReader(open(sys.argv[1])))[-1]
e = math.hypot(float(r['gx']) - float(r['x']), float(r['gy']) - float(r['y'])) * 100
print(f'square from /robot_pose: final position error {e:.2f} cm')
sys.exit(0 if e <= 1.5 else 1)
PY
}
t_no_leftovers() {
    cleanup; sleep 2
    local n
    n=$(pgrep -af "lab3_controllers/(motion|yaw)_controller" | grep -v pgrep | wc -l)
    echo "leftover controller processes: $n"
    [ "$n" -eq 0 ]
}

# ---------------------------------------------------------------- run
# shellcheck disable=SC1091
src /opt/ros/humble/setup.bash
ros2 daemon stop > /dev/null 2>&1 || true

echo "##### 1. static / unit";   run_test "unit tests (pytest)" t_unit
echo "##### 2. build";           run_test "fresh colcon build"  t_build
# shellcheck disable=SC1091
[ -f "$WS/install/setup.bash" ] && src "$WS/install/setup.bash"
if [ "$FAIL" -eq 0 ]; then
    echo "##### 3. safety (no Gazebo)"; run_test "safety tests (stale odom, shutdown zeros, invalid params)" t_safety
    run_test "launch files give the right state/command topics (config file vs override; hardware.yaml -> /cmd_vel_tracker)" t_launch_params
    run_test "p_controller node: heading, drive, pose on an unknown robot over ROS topics; live coefficients; stale stop" t_p_controller
    run_test "reference controller: reference from a topic (pose, yaw), coefficients changed while running, invalid ones rejected" t_reference
    echo "##### 4. Gazebo integration"; run_test "Gazebo up, /odom and cmd topic present" t_gazebo_up
    if [ "$FAIL" -eq 0 ] && [ -n "${ONLY_LAB:-}" ]; then
        # ONLY_LAB=1 bash run_all.sh <dir>: just the professor-robot emulation tests (faster than the full suite)
        run_test "lab_robot.yaml: square through the relay (final <= 2 cm / 1 deg, cross-track <= 3.5 cm incl. Gazebo's 1.3 mm/s drift)" t_lab_square
        run_test "lab_robot.yaml: yaw references through the relay (final error <= 1 deg)" t_lab_yaw
        run_test "P controller, circle of 1 m diameter (stop_turn): max distance <= 1.5 cm, final <= 1.5 cm" t_circle_p stop_turn
        run_test "P controller, circle of 1 m diameter (pure_pursuit): max distance <= 1.5 cm, final <= 1.5 cm" t_circle_p pure_pursuit
        run_test "symbolic controller, circle of 1 m diameter: completes, stays in the +-8 cm tube" t_symbolic
        run_test "response measurement tool: coasting = Gazebo alone + emulated plant (+-0.3 s), deadband none / 0.04 / 0.20" t_measure
    elif [ "$FAIL" -eq 0 ]; then
        run_test "yaw.launch.py reaches its reference"    t_yaw_launch
        run_test "yaw validation (5 references)"          t_yaw_validate
        run_test "motion.launch.py completes default path" t_motion_launch
        echo "##### 5. controller simulation"
        for shape in square circle; do for strat in stop_turn polar pure_pursuit; do
            run_test "run $shape/$strat completes" t_run "$shape" "$strat"
        done; done
        run_test "numeric acceptance thresholds" t_check
        echo "##### 5b. repeatability and presets"
        run_test "repeatability: ${REPEATS:-10} identical stop_turn square runs" t_repeat
        run_test "robust.yaml preset loads and completes" t_robust_preset
        echo "##### 5d. circle of 1 m diameter: P controllers and symbolic controller"
        run_test "P controller, circle of 1 m diameter (stop_turn): max distance <= 1.5 cm, final <= 1.5 cm" t_circle_p stop_turn
        run_test "P controller, circle of 1 m diameter (pure_pursuit): max distance <= 1.5 cm, final <= 1.5 cm" t_circle_p pure_pursuit
        run_test "symbolic controller, circle of 1 m diameter: completes, stays in the +-8 cm tube" t_symbolic
        run_test "response measurement tool: coasting = Gazebo alone + emulated plant (+-0.3 s), deadband none / 0.04 / 0.20" t_measure
        echo "##### 5c. professor-robot emulation (command filter + slow identified plant in front of Gazebo)"
        run_test "lab_robot.yaml: square through the relay (final <= 2 cm / 1 deg, cross-track <= 3.5 cm incl. Gazebo's 1.3 mm/s drift)" t_lab_square
        run_test "lab_robot.yaml: yaw references through the relay (final error <= 1 deg)" t_lab_yaw
        run_test "evaluation plots generated"    t_evaluate
        echo "##### 6. calibration tool"
        run_test "calibration tool plausible"    t_calib
        run_test "state-source inspector on /odom and /robot_pose" t_state_source
        run_test "square driven from the PoseStamped topic /robot_pose" t_pose_topic_run
    fi
else
    echo "SKIPPED integration tests because build/unit tests failed"
fi
run_test "no leftover controller processes" t_no_leftovers

echo
echo "================ SUMMARY ================"
echo "passed: $PASS   failed: $FAIL"
for f in "${FAILED[@]:-}"; do [ -n "$f" ] && echo "  FAILED: $f"; done
mkdir -p "$LAB/results/acceptance" && cp "$OUT"/*.csv "$OUT"/*.png "$OUT"/*.txt "$LAB/results/acceptance/" 2>/dev/null
[ "$FAIL" -eq 0 ] && { echo "ALL TESTS PASSED"; exit 0; } || { echo "SOME TESTS FAILED"; exit 1; }
