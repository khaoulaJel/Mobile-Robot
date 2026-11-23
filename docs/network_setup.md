# Network and bring-up: Raspberry Pi + laptop (WSL)

Setup that was made to work on 2026-11-23 (Windows 11 laptop with WSL2 Ubuntu 22.04, Raspberry Pi with Ubuntu 22.04, ROS 2 Humble,
CycloneDDS, both on the same Wi-Fi/LAN). Replace the IP addresses with your own (`hostname -I` on the Pi, `ipconfig` on Windows).

## 1. WSL must share the LAN address (once)

WSL2 defaults to NAT: the Pi cannot reach it. Create `C:\Users\<you>\.wslconfig`:

```
[wsl2]
networkingMode=mirrored
```

then `wsl --shutdown` in a Windows terminal and reopen WSL. `wslinfo --networking-mode` must print `mirrored`, and `hostname -I` in WSL shows the LAN address.

## 2. Windows firewall (once, administrator PowerShell)

Inbound UDP from the Pi is blocked by default (the Pi's ROS discovery never arrives). Replace the address with the Pi's:

```powershell
New-NetFirewallRule -DisplayName "Robot Pi to laptop (ROS 2)" -Direction Inbound -Protocol UDP -RemoteAddress 192.168.0.112 -Action Allow -Profile Any
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

Test: a UDP packet sent from the Pi to the laptop must arrive (`python3 -c` with a socket, or any UDP echo test).

## 3. CycloneDDS with fixed peers (both machines)

Multicast discovery did not work between the Pi and WSL, and with several interfaces every node was discovered twice. Fix: one interface, no
multicast, the other machine as the only peer.

Laptop `~/cdds.xml` (Pi at 192.168.0.112, laptop at 192.168.0.110):

```xml
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface address="192.168.0.110"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
      <Peers><Peer address="192.168.0.112"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

The Pi's `~/cdds.xml` is the same with the two addresses swapped. In every terminal on both machines:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file://$HOME/cdds.xml
```

## 4. Install and build

Pi (`sudo apt install`): `libserial-dev ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-rmw-cyclonedds-cpp ros-humble-twist-mux
ros-humble-xacro ros-humble-robot-state-publisher python3-colcon-common-extensions python3-serial`; the user must be in the `dialout` group.

```bash
cd pi_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

Laptop (WSL, not conda): `cd laptop_ws && ./install_dependencies.sh && ./build.sh`. Then the controllers: `colcon build --symlink-install --base-paths src`
in the repository root.

## 5. Start order

```bash
# Pi (no lidar plugged in: lidar:=false, otherwise the missing /dev/ttyUSB0 shuts the whole launch down)
source ~/Mobile-Robot/pi_ws/install/setup.bash
ros2 launch articubot_one hardware.launch.py lidar:=false

# laptop
cd laptop_ws && ./run_desktop.sh
```

Check from the laptop: `ros2 topic hz /diff_cont/odom` (about 25-40 Hz, no gaps above 0.3 s) and `ros2 node list | sort | uniq -d` (must print nothing).
Both machines also need synchronised clocks.

## 6. Arduino check without ROS

`python3-serial` and 57600 baud on `/dev/ttyACM0`: the command `e` returns the encoder counts (`0 0` at rest), `m 0 0` returns `OK` and moves nothing.

## Known behaviour

- Stopping a `ros2 launch` that was started with `nohup` needs SIGTERM (`pkill -TERM -f "ros2 launch"`); SIGINT is ignored. Child processes can remain
  (`ros2_control_node`, `rviz2`, `slam_toolbox`): stop them too before restarting.
- The launch files need executable scripts (`chmod +x` is stored in git).
