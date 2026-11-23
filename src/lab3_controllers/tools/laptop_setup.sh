#!/bin/bash
# Run on the LAPTOP (WSL), once, and again whenever an IP address changes:
#
#     bash laptop_setup.sh <PI_IP> [ROS_DOMAIN_ID]          (domain default 12: our own, not the shared 0)
#
# It writes ~/cdds.xml (CycloneDDS: this interface, the Pi as peer, multicast only for local discovery), puts the ROS
# environment in ~/.bashrc (idempotent block) and creates ~/start_laptop_stack.sh (the professor's laptop stack).
# The same domain number must be set on the Pi (bash pi_setup.sh <LAPTOP_IP> <domain>).
# Set HOME_OVERRIDE=<dir> to test without touching the real home directory.
set -u
PI_IP="${1:?usage: bash laptop_setup.sh <PI_IP> [ROS_DOMAIN_ID]}"
DOMAIN="${2:-12}"
H="${HOME_OVERRIDE:-$HOME}"
mkdir -p "$H"
if [ "$DOMAIN" = "0" ]; then echo "domain 0 is shared with every robot on the Wi-Fi: choose another number (1-100)" >&2; exit 1; fi
LAPTOP_IP="${LAPTOP_IP_OVERRIDE:-$(hostname -I | awk '{print $1}')}"
echo "laptop address: $LAPTOP_IP   Pi address: $PI_IP   ROS domain: $DOMAIN"
if command -v wslinfo > /dev/null 2>&1 && [ "$(wslinfo --networking-mode 2>/dev/null)" != "mirrored" ]; then
  echo "WARNING: WSL is not in mirrored networking mode: the Pi cannot reach WSL (docs/network_setup.md section 1)"
fi

cat > "$H/cdds.xml" <<EOF
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface address="$LAPTOP_IP"/></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="$PI_IP"/>
        <Peer address="$LAPTOP_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

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

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cat > "$H/start_laptop_stack.sh" <<EOF
#!/bin/bash
# the professor's laptop stack (robot_description, EKF, SLAM, twist_mux, joystick, RViz). Leave it running.
export ROS_DOMAIN_ID=$DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 CYCLONEDDS_URI=file://\$HOME/cdds.xml
cd "$REPO/laptop_ws" && exec ./run_desktop.sh
EOF
chmod +x "$H/start_laptop_stack.sh"
echo "written: $H/cdds.xml, $H/.bashrc block, $H/start_laptop_stack.sh"
echo "Open a NEW terminal (or: source ~/.bashrc) so the domain applies, then: bash ~/start_laptop_stack.sh"
echo "Then:  bash $REPO/src/lab3_controllers/tools/robot_session.sh check"
