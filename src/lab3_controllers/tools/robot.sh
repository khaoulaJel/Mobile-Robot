#!/bin/bash
# ONE COMMAND for the real robot:   bash robot.sh guide   (walks through the whole test day, step by step)
#
# Single commands. Run in WSL, after pi_setup.sh and laptop_setup.sh have been run for the present addresses.
#
#   bash robot.sh                 start what is not running, run the whole test series, make and open the report
#   bash robot.sh quick           the same with a short series
#   bash robot.sh pi2             the heading pi/2 six times from different angles (--compare adds plain P)
#   bash robot.sh identify turn   measure how far short pulses really move the robot
#   bash robot.sh turn 20         one test (also: drive 0.2 | heading 90, with --controller basic, --set heading.kp=1.5, --note x)
#   bash robot.sh path square --controller stop_turn      one path (path shapes = list; path series --shapes a,b)
#   bash robot.sh lidar           is /scan and the map position ready?  (moves nothing)
#   bash robot.sh results         rebuild and open the report
#   bash robot.sh status          is everything running and is exactly one robot visible?  (moves nothing)
#   bash robot.sh start           start the Pi driver and the laptop programs               (moves nothing)
#   bash robot.sh stop            stop the controllers, the laptop programs and the Pi driver
#
# The Pi address is the first peer in ~/cdds.xml; the login key is ~/.ssh/robot_key.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ROS 2 Humble needs the system Python. A terminal with conda active ("(base)") would give it conda's, which cannot load
# rclpy: take conda out of the search path for this script and everything it starts (your terminal is not changed).
PATH=$(printf '%s' "$PATH" | tr ':' '\n' | grep -vi conda | paste -sd: -)
export PATH
unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV
source "$HOME/robot_env.sh" || { echo "~/robot_env.sh missing: run laptop_setup.sh <PI_IP> first" >&2; exit 1; }
export ROBOT_LOGBOOK="${ROBOT_LOGBOOK:-$HOME/robot_logbook}"
PI_IP=$(grep -oE 'Peer address="[0-9.]+"' "$HOME/cdds.xml" | head -1 | grep -oE '[0-9]+(\.[0-9]+){3}')
LAPTOP_IP=$(grep -oE 'NetworkInterface address="[0-9.]+"' "$HOME/cdds.xml" | grep -oE '[0-9]+(\.[0-9]+){3}')
on_pi() { ssh -n -i"$HOME/.ssh/robot_key" -o BatchMode=yes -o ConnectTimeout=8 "ubuntu@$PI_IP" "$@"; }

start() {
    if ! hostname -I | tr ' ' '\n' | grep -qx "$LAPTOP_IP"; then
        echo "This laptop no longer has the address $LAPTOP_IP: join the robot's network (hotspot, VPN off)," >&2
        echo "or run laptop_setup.sh and pi_setup.sh again with the new addresses." >&2
        exit 1
    fi
    if ! on_pi true; then
        echo "Cannot log in to the Pi at $PI_IP: is it powered and on the same network?" >&2
        exit 1
    fi
    if on_pi 'pgrep -f "[r]os2_control_node" > /dev/null'; then echo "Pi driver: already running"; else
        echo "Pi driver: starting"
        on_pi 'setsid nohup bash $HOME/start_robot.sh > $HOME/robot_driver.log 2>&1 < /dev/null &'
        if ! on_pi 'for i in $(seq 1 60); do pgrep -f "[r]os2_control_node" > /dev/null && exit 0; sleep 1; done; exit 1'; then
            echo "The Pi driver did not start. See ~/robot_driver.log on the Pi." >&2
            exit 1
        fi
    fi
    if pgrep -f "[t]wist_mux/twist_mux" > /dev/null; then echo "laptop programs: already running"; else
        echo "laptop programs: starting"
        setsid nohup bash "$HOME/start_laptop_stack.sh" > "$HOME/laptop_stack.log" 2>&1 < /dev/null &
    fi
}

status() {
    echo "Pi $PI_IP, laptop $LAPTOP_IP, ROS domain $ROS_DOMAIN_ID"
    on_pi 'ls /dev/ttyACM0 > /dev/null 2>&1 && echo "Arduino: seen" || echo "Arduino: NOT SEEN"; ls /dev/ttyUSB0 > /dev/null 2>&1 && echo "LiDAR: seen" || echo "LiDAR: not connected"; pgrep -f "[r]os2_control_node" > /dev/null && echo "Pi driver: running" || echo "Pi driver: NOT running"'
    pgrep -f "[t]wist_mux/twist_mux" > /dev/null && echo "laptop programs: running" || echo "laptop programs: NOT running"
    echo "waiting for odometry ..."
    for _ in 1 2 3 4 5 6; do
        INFO=$(timeout 25 ros2 topic info /diff_cont/odom --no-daemon --spin-time 6 2>/dev/null)
        N=$(echo "$INFO" | awk '/Publisher count/ {print $3}')
        [ "${N:-0}" != "0" ] && break
    done
    case "${N:-0}" in
        1) echo "odometry publishers: 1  (good: exactly one robot)" ;;
        0) echo "odometry publishers: 0  (see ~/robot_driver.log on the Pi and ~/laptop_stack.log here)"; return 1 ;;
        *) echo "odometry publishers: $N  (STOP: more than one robot on this domain)"; return 1 ;;
    esac
}

open_report() {
    local f="$ROBOT_LOGBOOK/report.html"
    [ -f "$f" ] && command -v explorer.exe > /dev/null && explorer.exe "$(wslpath -w "$f")"
    return 0
}

tool() { /usr/bin/python3 "$HERE/robot_test.py" "$@"; }

CMD="${1:-all}"
[ $# -gt 0 ] && shift
case "$CMD" in
    all)     start && status && { tool suite "$@"; open_report; } ;;
    quick)   start && status && { tool suite --quick "$@"; open_report; } ;;
    pi2)     start && status && { tool pi2 "$@"; open_report; } ;;
    turn|drive|heading) start && tool "$CMD" "$@"; code=$?; [ -f "$ROBOT_LOGBOOK/logbook.csv" ] && tool report > /dev/null; exit $code ;;
    results) tool report && open_report ;;
    identify) start && /usr/bin/python3 "$HERE/robot_identify.py" "$@" ;;   # how far each short command really moves it
    show|hand) tool "$CMD" "$@" ;;
    guide)   /usr/bin/python3 "$HERE/robot_guide.py" ;;
    path)    if [ "${1:-}" = shapes ]; then /usr/bin/python3 "$HERE/robot_path.py" shapes; else
                 start && { /usr/bin/python3 "$HERE/robot_path.py" "$@"; code=$?; exit $code; }; fi ;;
    lidar)   start && /usr/bin/python3 "$HERE/lidar_check.py" ;;
    status)  status ;;
    start)   start && status ;;
    stop)
        pkill -TERM -f "lib/lab3_controllers/[p]_controller"; pkill -TERM -f "lib/lab3_controllers/[r]eference_controller"; sleep 1
        # a launch started in the background ignores Ctrl+C signals: end its whole process group
        END='for p in $(pgrep -f "$1"); do kill -TERM -- -$(ps -o pgid= -p $p | tr -d " ") 2>/dev/null; done'
        bash -c "$END" end "[d]esktop.launch.py"
        on_pi "bash -c '$END' end '[h]ardware.launch.py'"
        sleep 5
        pgrep -f "[t]wist_mux/twist_mux" > /dev/null && echo "laptop programs: still running" || echo "laptop programs: stopped"
        on_pi 'pgrep -f "[r]os2_control_node" > /dev/null && echo "Pi driver: still running" || echo "Pi driver: stopped"' ;;
    *) sed -n 2,20p "$HERE/robot.sh"; exit 2 ;;
esac
