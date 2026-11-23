#!/bin/bash
# Run ON THE RASPBERRY PI, once, and again whenever an IP address changes:
#
#     bash pi_setup.sh <LAPTOP_IP> [ROS_DOMAIN_ID]          (domain default 12: our own, not the shared 0)
#
# It (1) detects the Pi's own IP, (2) writes ~/cdds.xml (CycloneDDS: this interface, the laptop as peer, multicast only for
# local discovery), (3) sets the ROS environment in ~/.bashrc (idempotent block), (4) creates ~/start_robot.sh that starts the
# robot driver, (5) checks the serial port to the Arduino and talks to it (reads the encoders, sends a stop: nothing moves).
# Set HOME_OVERRIDE=<dir> to test without touching the real home directory.
set -u
LAPTOP_IP="${1:?usage: bash pi_setup.sh <LAPTOP_IP> [ROS_DOMAIN_ID]}"
DOMAIN="${2:-12}"
H="${HOME_OVERRIDE:-$HOME}"
mkdir -p "$H"
if [ "$DOMAIN" = "0" ]; then echo "domain 0 is shared with every robot on the Wi-Fi: choose another number (1-100)" >&2; exit 1; fi
PI_IP="${PI_IP_OVERRIDE:-$(hostname -I | awk '{print $1}')}"
echo "Pi address: $PI_IP   laptop address: $LAPTOP_IP   ROS domain: $DOMAIN"

cat > "$H/cdds.xml" <<EOF
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface address="$PI_IP"/></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="$LAPTOP_IP"/>
        <Peer address="$PI_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

# environment block in .bashrc (replaced, not duplicated)
touch "$H/.bashrc"
sed -i '/# >>> robot ros >>>/,/# <<< robot ros <<</d' "$H/.bashrc"
cat >> "$H/.bashrc" <<EOF
# >>> robot ros >>>
export ROS_DOMAIN_ID=$DOMAIN
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file://\$HOME/cdds.xml
# <<< robot ros <<<
EOF

cat > "$H/start_robot.sh" <<EOF
#!/bin/bash
# starts the robot driver (no lidar plugged in: lidar:=false). Leave it running; stop with Ctrl+C.
source /opt/ros/humble/setup.bash
source \$HOME/Mobile-Robot/pi_ws/install/setup.bash
export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 CYCLONEDDS_URI=file://\$HOME/cdds.xml
exec ros2 launch articubot_one hardware.launch.py lidar:=false
EOF
chmod +x "$H/start_robot.sh"
echo "written: $H/cdds.xml, $H/.bashrc block, $H/start_robot.sh"

# checks
[ -n "${HOME_OVERRIDE:-}" ] && exit 0
echo "--- checks"
id -nG | grep -qw dialout && echo "dialout group: ok" || echo "WARNING: not in the dialout group: sudo usermod -aG dialout \$USER, then log in again"
PORT=$(ls /dev/ttyACM* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then echo "WARNING: no /dev/ttyACM*: the Arduino is not seen (USB cable, power)"; exit 0; fi
python3 - "$PORT" <<'PYEOF'
import sys, time
try:
    import serial
except ImportError:
    print('python3-serial missing: sudo apt install python3-serial'); sys.exit(0)
with serial.Serial(sys.argv[1], 57600, timeout=1.0) as s:
    time.sleep(2.0); s.reset_input_buffer()
    for cmd in (b'e\r', b'm 0 0\r'):
        s.write(cmd); print(f'Arduino {cmd!r} ->', repr(s.readline().decode(errors="replace").strip()))
PYEOF
echo "Start the robot driver with:  bash ~/start_robot.sh"
