# Livox Mid-360 / Mid-360S — ROS2 (Humble) driver setup from scratch

Tested working on Ubuntu 22.04 + ROS2 Humble, with the lidar connected via direct
Ethernet cable.

## 1. Install prerequisites

```bash
sudo apt install -y cmake build-essential
```

## 2. Create a clean workspace and clone sources

Use the **upstream** repos, not vendor/robot-specific forks — older forks may predate
newer lidar variants (e.g. Mid-360S).

```bash
mkdir -p ~/livox_test_ws/src && cd ~/livox_test_ws/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

## 3. Build & install Livox-SDK2 (system-wide, one-time)

```bash
cd ~/livox_test_ws/src/Livox-SDK2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install && sudo ldconfig
```

## 4. Configure networking

- Connect the lidar via Ethernet (direct cable or through a switch).
- Give your NIC a static IP on the lidar's subnet, e.g. `192.168.0.101/24` if the
  lidar is on `192.168.0.x`.
- Find the lidar's actual IP. It keeps whatever IP it was last configured with
  across power cycles, so check `ip neigh` after it's been powered on a few
  seconds, or ping-sweep the subnet.
- If `ufw`/a firewall is active, it may block the UDP traffic — check
  `sudo ufw status`, and if active, allow the lidar's IP on the Livox port range
  (56000, 56100-56500):
  ```bash
  sudo ufw allow from <lidar_ip> to any port 56000 proto udp
  sudo ufw allow from <lidar_ip> to any port 56100:56500 proto udp
  ```

## 5. Identify the exact model (critical — do not skip)

The datasheet/marketing name (e.g. "Mid-360") does not always match the JSON
config key the SDK expects. Confirm the real `dev_type` reported by the device
before writing any config:

```bash
cd ~/livox_test_ws/src/Livox-SDK2/build
```

Create a throwaway config (only the `host_net_info` IP matters here) using any of
the sample JSON files under `Livox-SDK2/samples/*/`, then run:

```bash
./samples/livox_lidar_quick_start/livox_lidar_quick_start /path/to/throwaway_config.json
```

Watch for a line like:

```
Handle detection data, handle:167815360, dev_type:35, sn:ARMCP760038525, cmd_port:56100
```

Map `dev_type` to the model:

| dev_type | Model              |
|----------|--------------------|
| 1        | Mid-40             |
| 2        | Tele-15            |
| 3        | Horizon            |
| 6        | Mid-70             |
| 7        | Avia               |
| 9        | Mid-360            |
| 10       | Industrial HAP     |
| 15       | HAP                |
| 16       | PA                 |
| 35       | **Mid-360S**       |

If nothing prints within ~10s but `sudo tcpdump -i <iface> udp port 56000` shows
two-way traffic with the lidar, the JSON model key doesn't match the detected
`dev_type` — the SDK silently drops unrecognized devices with **no error message**.

## 6. Set the config file with the correct model key and IPs

In `~/livox_test_ws/src/livox_ros_driver2/config/`, pick the file matching the
detected model (e.g. `MID360s_config.json` for dev_type 35, `MID360_config.json`
for dev_type 9). Edit only:

- `host_net_info` IP(s) → your host's static IP
- `lidar_configs[0].ip` → the lidar's detected IP

Note the two schema variants used by different config files:
- `MID360_config.json` style: `host_net_info` is an **object** with per-field IPs
  (`cmd_data_ip`, `push_msg_ip`, ...).
- `MID360s_config.json` style: `host_net_info` is an **array** of objects with a
  single `host_ip` field.
Use whichever schema the specific file already has — don't mix them.

## 7. Build the ROS2 driver

```bash
source /opt/ros/humble/setup.bash
cd ~/livox_test_ws/src/livox_ros_driver2
./build.sh humble
```

## 8. Launch

```bash
source /opt/ros/humble/setup.bash
source ~/livox_test_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360s_launch.py   # pick the launch file matching your model
```

Verify data is flowing:

```bash
ros2 topic list                 # expect /livox/lidar and /livox/imu
ros2 topic hz /livox/lidar       # ~10 Hz by default
ros2 topic hz /livox/imu         # ~200 Hz by default
```

## Troubleshooting checklist

- No topics at all, node logs stop after "Init lds lidar success!" →  wrong model
  key in config (see step 5).
- `tcpdump` shows the lidar streaming point/imu data to your host IP:port but no
  ROS topics → same as above, or check `sudo ufw status` for a blocking firewall.
- Process crashes (SIGSEGV) right after removing the
  `DisableLivoxSdkConsoleLogger()` call in `lds_lidar.cpp` → don't remove it, this
  build's SDK logger isn't safe to enable that way; use the raw
  `livox_lidar_quick_start` sample instead for verbose diagnostics.
- Lidar physically has an LED/spinning but no network traffic at all → check it
  has its own DC power connected (Ethernet-only cables typically carry data, not
  power, on the Mid-360 series).
