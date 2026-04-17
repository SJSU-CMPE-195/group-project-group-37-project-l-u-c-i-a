# Architecture Options

The Jetson Nano runs vision and must send detection results to the Raspberry Pi,
which then issues drive commands to the Roomba. This document evaluates the
communication layer options between the two boards.

---

## Option A — Plain TCP Socket

The Jetson opens a TCP server; the Pi connects as a client. Detection frames are
sent as length-prefixed JSON or raw bytes.

**Pros:**
- Zero extra dependencies
- Works over Ethernet or Wi-Fi

**Cons:**
- Manual message framing required
- No pub/sub; adding a third subscriber (e.g. a laptop monitor) requires extra work

---

## Option B — ZMQ (ZeroMQ)

ZMQ is a lightweight messaging library. The Jetson publishes detection frames on a
PUB socket; the Pi subscribes on a SUB socket. Both sides use `pyzmq`.

**Pros:**
- Single `pip install pyzmq` on each board
- Handles framing and buffering automatically
- Pub/sub is easy to extend (laptop can subscribe for live monitoring)
- No broker required
- Familiar pattern if ROS2 is adopted later

**Cons:**
- One extra dependency per board

---

## Option C — MQTT (e.g. Mosquitto broker on Pi)

A lightweight pub/sub protocol. The Pi runs a Mosquitto broker; the Jetson publishes
detection topics; the Pi subscribes to them.

**Pros:**
- Clean pub/sub model
- Well-supported, good Python client (`paho-mqtt`)
- Good for IoT-style monitoring

**Cons:**
- Requires running a broker process on the Pi
- Higher latency than direct socket/ZMQ for local communication
- Adds operational complexity

---

## Option D — ROS2

The full robotics middleware stack. The ZED 2i has an official ROS2 wrapper
(`zed-ros2-wrapper`) that publishes depth, point cloud, and detection topics natively.

**Pros:**
- First-class ZED SDK integration via official wrapper
- Rich ecosystem: nav2, rviz2, tf2, etc.
- Best long-term path if the project grows
- Standard in robotics research

**Cons:**
- Significant setup overhead on both boards
- Steep learning curve if the team hasn't used ROS2 before
- May be more than the project scope requires

---

## Comparison Table

| Option | Dependencies | Broker needed | Pub/Sub | Complexity | Best for |
|--------|-------------|---------------|---------|------------|----------|
| Plain TCP | None | No | No | Low | Quick prototype |
| ZMQ | pyzmq | No | Yes | Low | Quick prototype with pub/sub |
| MQTT | paho-mqtt + Mosquitto | Yes (on Pi) | Yes | Medium | IoT-style monitoring |
| ROS2 | Full ROS2 stack | No (DDS) | Yes | High | **Selected — see below** |

---

## Decision: ROS2

ROS2 was selected as the communication and integration layer for the following reasons:

1. **Stereolabs provides an official `zed-ros2-wrapper`** that publishes depth, point
   cloud, and object detection topics natively. No custom ZED integration code needed.

2. **`rplidar_ros2`** is the standard driver for the RPLidar A2. Combined with
   `slam_toolbox`, this gives 2D SLAM out of the box.

3. **Nav2** (the ROS2 navigation stack) ties everything together: it consumes the
   LiDAR map for path planning and the ZED object detections as dynamic obstacles in
   its costmap. This is the core of the autonomous navigation requirement.

4. **Inter-board communication is solved automatically.** ROS2 uses DDS (Data
   Distribution Service) for discovery. Both boards connect over a direct Gigabit
   Ethernet cable, set the same `ROS_DOMAIN_ID`, and all topics are visible across
   both boards with no manual socket or broker setup.

5. **`rviz2`** gives the team a live visualization of the map, robot pose, LiDAR scan,
   and ZED detections from a laptop — essential for debugging.

The setup investment is real, but every major subsystem (LiDAR, ZED, SLAM, navigation)
has a production-grade ROS2 package that handles the hard parts.
