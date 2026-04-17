# Proposed Architecture

## Decision

Use **ROS2 over direct Gigabit Ethernet** as the integration layer.

See [architecture-options.md](architecture-options.md) for why ROS2 was chosen over
ZMQ, MQTT, and plain TCP.

---

## Physical Wiring

```
┌──────────────────────────────────────────────────┐
│                  Raspberry Pi                    │
│                                                  │
│  [X1202 UPS HAT] ←── power (supplies Pi)         │
│                                                  │
│  USB ←── Roomba 650                              │
│            USB-to-serial cable                   │
│            → /dev/roomba  (udev alias)           │
│            115200 baud                           │
│                                                  │
│  USB ←── RPLidar A2                              │
│            USB-UART adapter (included)           │
│            → /dev/rplidar  (udev alias)          │
│            115200 baud                           │
│                                                  │
│  I2C + GPIO BCM 6, 16 ←── X1202 UPS HAT         │
│                                                  │
│  eth0 ◄─────── direct Cat5e/6 cable ────────────►eth0
└──────────────────────────────────────────────────┘
                                                   │
┌──────────────────────────────────────────────────┤
│                  Jetson Nano                     │
│                                                  │
│  USB 3.0 (blue port) ←── ZED 2i                  │
│                                                  │
│  Barrel jack (5.5mm/2.1mm) ←── power bank       │
│    5V / 4A minimum                               │
│    J48 jumper must be shorted                    │
└──────────────────────────────────────────────────┘
```

### USB Device Naming (udev rules)

The Pi has two USB serial devices: the Roomba and the RPLidar A2. Linux assigns
`/dev/ttyUSB0` and `/dev/ttyUSB1` at boot, but the order is not guaranteed and
can change if a device is replugged.

Set up udev rules to assign fixed, permanent names based on USB vendor/product ID:

```bash
# Find the IDs of each device
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
udevadm info -a -n /dev/ttyUSB1 | grep -E 'idVendor|idProduct'
```

Then create `/etc/udev/rules.d/99-lucia.rules`:

```
# Roomba 650 (CP210x USB-serial)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="roomba"

# RPLidar A2 (CP2102 USB-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<serial_number>", SYMLINK+="rplidar"
```

> Note: if both devices share the same vendor/product ID (both use CP210x chips),
> differentiate them by the `serial` attribute from `udevadm info`.

After adding the rules: `sudo udevadm control --reload && sudo udevadm trigger`

Scripts and ROS2 launch files should reference `/dev/roomba` and `/dev/rplidar`
rather than `/dev/ttyUSBX`.

---

## Inter-board Communication

Both boards are connected by a **direct Ethernet cable** — no router or switch needed.

### One-time network setup

**Raspberry Pi** (`/etc/dhcpcd.conf` or netplan):
```
static ip_address=192.168.1.1/24
```

**Jetson Nano** (`/etc/netplan/...` or nmcli):
```
static ip_address=192.168.1.2/24
```

### ROS2 domain setup (add to `~/.bashrc` on both boards)

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Once both boards are on the same domain, ROS2 DDS (FastDDS) handles peer discovery
automatically. The Pi can subscribe to any topic the Jetson publishes and vice versa
with no additional configuration.

---

## ROS2 Software Stack

### Raspberry Pi nodes

| Package | Publishes / Subscribes | Purpose |
|---------|----------------------|---------|
| `rplidar_ros2` | → `/scan` | Drives RPLidar A2 |
| `slam_toolbox` | `/scan` → `/map`, `/tf` | 2D SLAM + localization |
| `nav2` | `/map`, `/tf`, ZED topics → `/cmd_vel` | Path planning + obstacle avoidance |
| `roomba_bridge` *(custom)* | `/cmd_vel` → serial | Translates Nav2 output to `RoombaOI.drive_direct()` |

### Jetson Nano nodes

| Package | Publishes | Purpose |
|---------|-----------|---------|
| `zed-ros2-wrapper` | `/zed/...` topics | Drives ZED 2i; publishes depth, objects, odometry |

### Key ZED topics consumed by the Pi

| Topic | Consumed by | Purpose |
|-------|------------|---------|
| `/zed/zed_node/obj_det/objects` | Nav2 costmap | Dynamic obstacles (people, objects) |
| `/zed/zed_node/depth/depth_registered` | Nav2 *(optional)* | Dense depth as additional costmap layer |
| `/zed/zed_node/odom` | slam_toolbox *(optional)* | Visual odometry to supplement LiDAR |

---

## Data Flow

```
[Raspberry Pi]                              [Jetson Nano]

RPLidar A2                                  ZED 2i
  │ /dev/rplidar                              │ USB 3.0
  ▼                                           ▼
rplidar_ros2                            zed-ros2-wrapper
  │ /scan                                     │
  ▼                                           ├── /zed/.../objects ────────────┐
slam_toolbox                                  └── /zed/.../odom (optional) ──┐ │
  │ /map                                                                      │ │
  │ /tf (robot pose)                                                          │ │
  └────────────────────────► Nav2 ◄──────────────────────────────────────────┘─┘
                               │
                               │ /cmd_vel
                               ▼
                         roomba_bridge
                               │ serial /dev/roomba
                               ▼
                          Roomba 650
```

---

## Build Checklist

### Hardware

- [ ] Confirm J48 jumper is shorted on Jetson Nano
- [ ] Verify power bank outputs 5V/4A minimum (barrel jack, center-positive)
- [ ] Confirm Jetson and Pi are on separate power rails
- [ ] Run Cat5e/6 Ethernet cable between Pi and Jetson
- [ ] Confirm ZED 2i is in the blue USB 3.0 port on Jetson (not USB 2.0)
- [ ] Confirm Roomba USB-serial cable and RPLidar USB-UART adapter are both plugged into Pi
- [ ] Set up udev rules so `/dev/roomba` and `/dev/rplidar` are stable

### Raspberry Pi — Software

- [ ] Install ROS2 Humble
- [ ] Install `rplidar_ros2` and test scan output
- [ ] Install `slam_toolbox` and verify map building
- [ ] Install `nav2` and configure costmap layers
- [ ] Write `roomba_bridge` ROS2 node wrapping `roomba_oi.py`
- [ ] Configure Nav2 to ingest `/zed/zed_node/obj_det/objects` as dynamic obstacles
- [ ] Set static IP `192.168.1.1` on `eth0`
- [ ] Add `ROS_DOMAIN_ID=42` to `~/.bashrc`

### Jetson Nano — Software

- [ ] Flash JetPack 5.x (required for ZED SDK 4.x)
- [ ] Install ZED SDK 4.x and run self-test
- [ ] Install ROS2 Humble (ARM build)
- [ ] Install `zed-ros2-wrapper` and verify topics publish
- [ ] Set static IP `192.168.1.2` on `eth0`
- [ ] Add `ROS_DOMAIN_ID=42` to `~/.bashrc`

### Integration

- [ ] Confirm Pi can see Jetson's ROS2 topics (`ros2 topic list` from Pi)
- [ ] Calibrate TF transform between RPLidar and ZED 2i physical mount positions
- [ ] Set up `rviz2` on a laptop (same `ROS_DOMAIN_ID`) for live visualization
- [ ] Run end-to-end test: rover navigates to goal, avoids a person walking in front

---

## What Does Not Change

- `roomba_oi.py` — no modifications needed; `roomba_bridge` wraps it
- `control_panel_ssh.py` — stays intact for manual teleoperation and override
- All existing drive and demo scripts continue to work independently of ROS2

---

## Open Questions

- Should `roomba_bridge` enforce a hard stop on Roomba bump/cliff sensor triggers
  regardless of what Nav2 commands? (Recommended: yes, as a safety layer.)
- Do we fuse ZED visual odometry into `slam_toolbox`, or rely on LiDAR odometry
  alone? Fusion improves accuracy but adds complexity and a calibration step.
- If the Jetson goes offline mid-navigation, should Nav2 degrade gracefully to
  LiDAR-only obstacle avoidance, or stop and wait? Graceful degradation is safer
  for the stated use case (dynamic indoor environments).
