# Jetson Nano Deployment — Index

Live documentation of the Jetson Nano setup for Project L.U.C.I.A.
Written as we go — commands, actual output, and notes from real hardware.

---

## Files

| File | Phase | Status |
|------|-------|--------|
| [00-reflash.md](00-reflash.md) | Recovery mode, manual flash via flash.sh | ✅ Done |
| [01-hardware-check.md](01-hardware-check.md) | JetPack version, first boot setup | ✅ Done |
| [02-zed-sdk.md](02-zed-sdk.md) | ZED SDK 3.x installation and self-test | 🔄 In progress |
| [03-ros2-humble.md](03-ros2-humble.md) | ROS2 via Docker (Ubuntu 18.04 workaround) | ⏳ Pending |
| [04-zed-ros2-wrapper.md](04-zed-ros2-wrapper.md) | ZED ROS2 wrapper build and topic verification | ⏳ Pending |
| [05-network.md](05-network.md) | Static IP, Ethernet link to Pi, cross-board ping | ⏳ Pending |
| [06-cross-board-ros2.md](06-cross-board-ros2.md) | ROS2 topic visibility Pi ↔ Jetson | ⏳ Pending |

---

## Hardware Reference

| Item | Value |
|------|-------|
| Jetson Nano SSH (USB) | `ssh lucia@192.168.55.1` |
| Jetson static IP (Ethernet) | `192.168.1.2` |
| Pi static IP (Ethernet) | `192.168.1.1` |
| Hostname | `lucia-jetson` |
| Username | `lucia` |
| ROS_DOMAIN_ID | `42` |
| RMW | `rmw_fastrtps_cpp` |
| ZED port | Blue USB 3.0 only |
| JetPack | 4.6.1 / L4T R32.7.1 / Ubuntu 18.04 |
| CUDA | 10.2 |
| ZED SDK | 3.x (CUDA 10.2 compatible) |
| ROS2 distro | Humble via Docker (Ubuntu 18.04 has no native Humble packages) |
