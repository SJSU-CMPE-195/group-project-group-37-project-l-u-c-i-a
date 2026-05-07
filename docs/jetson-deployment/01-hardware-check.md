# Phase 1 — Hardware Check

Verify the Jetson is accessible and confirm the JetPack version before
installing anything. Everything downstream (ZED SDK version, ROS2 distro)
depends on this.

---

## Step 1 — SSH into Jetson over USB

Connect the Jetson to your laptop via USB, then:

```bash
ssh nvidia@192.168.55.1
```

**Result:** Connected, but original password was unknown. Could not log in.
See [00-reflash.md](00-reflash.md) for how we resolved this.

---

## Step 2 — JetPack version (post-reflash)

After reflashing with L4T R32.7.1, the Jetson runs:

- **L4T:** R32.7.1
- **OS:** Ubuntu 18.04.5 LTS
- **JetPack:** 4.6.1
- **CUDA:** 10.2

> ⚠️ This is JetPack 4.x, NOT 5.x as originally planned.
> This affects the ZED SDK version (must use 3.x, not 4.x) and ROS2 distro.
> See updated notes in [02-zed-sdk.md](02-zed-sdk.md) and [03-ros2.md](03-ros2.md).

---

## Step 3 — Serial console confirmed

Serial login accessible via:

```bash
screen /dev/ttyACM0 115200
```

---

## Step 4 — First boot setup wizard choices

| Prompt | Choice |
|--------|--------|
| Primary network interface | eth0 |
| Network configuration | Do not configure at this time |
| Hostname | `lucia-jetson` |
| Nvpmodel mode | MAXN (full performance) |
| Username | `lucia` |
| Password | `lucia-143-tomato` |

---

## Step 5 — System update

Connected Jetson to network via Ethernet (`eth0`, DHCP, got `192.168.0.21`):

```bash
sudo dhclient eth0
sudo apt update && sudo apt upgrade -y
```

Accepted `Y` for all config file prompts (nvidia-tegra.conf, nv-oem-config-post.sh,
nv-oem-config.sh). Selected `Yes` for Docker daemon auto-restart prompt.

Update applied: `nvidia-l4t-core 32.7.6`, kernel `4.9.337-tegra-32.7.6`, Docker, and others.

## Step 6 — Docker confirmed pre-installed

```bash
docker --version
# Docker version 20.10.21, build 20.10.21-0ubuntu1~18.04.3
```

Docker ships with JetPack — no separate install needed.

---

## Notes

- Original password was unknown — Jetson was reflashed from scratch (see [00-reflash.md](00-reflash.md))
- eMMC board (no SD card slot) — reflash required recovery mode via FC_REC pin
- USB power from laptop is sufficient to boot but a dedicated 5V/4A supply is recommended for sustained operation
- Docker 20.10.21 pre-installed via JetPack — no separate install needed
- Internet access via Ethernet to router (`192.168.0.21`) — USB device mode has no internet routing
