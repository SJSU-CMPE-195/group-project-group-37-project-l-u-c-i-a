# Jetson Nano Deployment — Index

Live documentation of the Jetson Nano setup for Project L.U.C.I.A.
Written as we go — commands, actual output, and notes from real hardware.

---

## Files

| File | Phase | Status |
|------|-------|--------|
| [01-hardware-check.md](01-hardware-check.md) | JetPack version, ZED USB detection | 🔄 In progress |
| [02-zed-sdk.md](02-zed-sdk.md) | ZED SDK 4.x installation and self-test | ⏳ Pending |
| [03-ros2-humble.md](03-ros2-humble.md) | ROS2 Humble install and workspace setup | ⏳ Pending |
| [04-zed-ros2-wrapper.md](04-zed-ros2-wrapper.md) | ZED ROS2 wrapper build and topic verification | ⏳ Pending |
| [05-network.md](05-network.md) | Static IP, Ethernet link to Pi, cross-board ping | ⏳ Pending |
| [06-cross-board-ros2.md](06-cross-board-ros2.md) | ROS2 topic visibility Pi ↔ Jetson | ⏳ Pending |

---

## Hardware Reference

| Item | Value |
|------|-------|
| Jetson Nano SSH (USB) | `ssh nvidia@192.168.55.1` |
| Jetson static IP (Ethernet) | `192.168.1.2` |
| Pi static IP (Ethernet) | `192.168.1.1` |
| ROS_DOMAIN_ID | `42` |
| RMW | `rmw_fastrtps_cpp` |
| ZED port | Blue USB 3.0 only |
| ROS2 distro | Humble (Ubuntu 22.04 / JetPack 5.x) |
