# Hardware Constraints

## Why the ZED 2i cannot run on the Raspberry Pi

The Stereolabs ZED 2i is a stereo depth camera that requires the **ZED SDK** to access
its core features: depth maps, point clouds, spatial mapping, and object detection.

The ZED SDK has a hard dependency on **NVIDIA CUDA**. The Raspberry Pi has no GPU and
no CUDA support, so the SDK cannot run on it at all.

Using the ZED 2i as a plain USB (UVC) camera on the Pi is technically possible but
throws away everything that makes the camera useful for this project — depth, tracking,
and all spatial features. It also leaves the Pi underpowered for real-time inference.

## Why the Jetson Nano handles vision

The Jetson Nano has a 128-core Maxwell GPU with full CUDA support. It is the minimum
viable NVIDIA platform for running the ZED SDK. It can handle:

- ZED SDK (depth, point clouds, spatial mapping)
- ZED's built-in object detection (runs on-device via CUDA)
- Custom detection models (YOLOv8-nano, etc.) if needed

## Why the Raspberry Pi stays as controller

The Pi is already wired to the Roomba 650 via serial (`/dev/ttyUSB0`) and has the
X1202 UPS connected over I2C + GPIO (BCM pins 6 and 16). All existing control scripts
target the Pi. Migrating serial and I2C wiring to the Jetson would be churn with no
benefit — the Pi is well-suited for low-level I/O.

## Why the RPLidar A2 and ZED 2i are complementary, not redundant

The RPLidar A2 (model A2M8) is a **2D** 360° scanner. It scans a single horizontal
plane at 10Hz, producing 800 distance+angle samples per sweep up to 12 meters. This
is well-suited for building a 2D occupancy map and localizing within it.

However, being 2D means it:
- Cannot identify *what* it detects (a person's legs and a table leg look identical)
- Misses anything above or below its scan plane entirely

The ZED 2i fills these gaps with 3D depth and labeled object detection. The two
sensors serve distinct roles and do not overlap.

## Summary

| Component | Connects to | Interface | Fixed device name | Role |
|-----------|------------|-----------|-------------------|------|
| Roomba 650 | Raspberry Pi | USB-serial cable | `/dev/roomba` | Drive platform |
| RPLidar A2 (A2M8) | Raspberry Pi | USB-UART adapter (included) | `/dev/rplidar` | 2D SLAM + localization |
| X1202 UPS HAT | Raspberry Pi | I2C + GPIO (BCM 6, 16) | — | Pi power management |
| ZED 2i | Jetson Nano | USB 3.0 (blue port) | — | 3D depth + object detection |
| Jetson Nano | Raspberry Pi | Gigabit Ethernet (direct Cat5e/6) | — | Vision compute node |
| Power bank (5V/4A) | Jetson Nano | Barrel jack 5.5mm/2.1mm (J48 shorted) | — | Jetson power |
