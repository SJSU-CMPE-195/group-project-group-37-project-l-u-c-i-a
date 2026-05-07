# Phase 0 — Reflash (Recovery)

We could not log into the Jetson — the original password was unknown.
This doc covers how we got into recovery mode and reflashed with a fresh L4T image.

---

## Hardware

- Jetson Nano 4GB (eMMC, no SD card)
- Connected to laptop via USB (USB device mode)
- Board revision: B01 (40-pin GPIO header, two camera connectors)

---

## Step 1 — Install screen and connect serial console

```bash
sudo apt install screen
screen /dev/ttyACM0 115200
```

Serial console confirmed working — showed `Ubuntu 18.04.5 LTS Nano ttyGS0` login prompt.
Password was unknown; all default passwords (`nvidia`, `jetson`, `ubuntu`, etc.) failed.

---

## Step 2 — Enter recovery mode via FC_REC pin

The Jetson Nano 4GB does not have labeled push buttons for recovery — it uses a
2-pin header labeled **FC_REC** flanked by two GND pins, located near the camera
connectors on the board.

**Procedure:**
1. Short **FC_REC to either adjacent GND pin** using a jumper wire or tweezers
2. While holding the short, unplug and replug the USB cable to reboot
3. Release the short

Confirm recovery mode on the laptop:

```bash
lsusb | grep -i nvidia
# Bus 003 Device 025: ID 0955:7f21 NVIDIA Corp. APX
```

`APX` = recovery mode confirmed.

---

## Step 3 — Install dependencies on laptop

```bash
sudo apt install libxml2-utils qemu-user-static
```

---

## Step 4 — Download L4T 32.7.1

From the NVIDIA developer site (L4T 32.7.1 Release Page), downloaded:

- `Jetson-210_Linux_R32.7.1_aarch64.tbz2` — BSP / Driver Package (264 MB)
- `Tegra_Linux_Sample-Root-Filesystem_R32.7.1_aarch64.tbz2` — Root Filesystem (1.4 GB)

---

## Step 5 — Extract and prepare

```bash
mkdir ~/jetson-flash && cd ~/jetson-flash
tar xf ~/Downloads/Jetson-210_Linux_R32.7.1_aarch64.tbz2
sudo tar xpf ~/Downloads/Tegra_Linux_Sample-Root-Filesystem_R32.7.1_aarch64.tbz2 \
  -C Linux_for_Tegra/rootfs/
sudo Linux_for_Tegra/apply_binaries.sh
```

---

## Step 6 — Flash

```bash
cd Linux_for_Tegra
sudo ./flash.sh jetson-nano-devkit-emmc mmcblk0p1
```

> Note: `mmcblk0boot0` and `jetson-nano-emmc` are both invalid — the correct
> combination is `jetson-nano-devkit-emmc` + `mmcblk0p1`.

**Output:**
```
*** The target t210ref has been flashed successfully. ***
Reset the board to boot from internal eMMC.
```

Flash took approximately 5 minutes.

---

## Step 7 — First boot

Jetson rebooted automatically. Reconnected serial console:

```bash
screen /dev/ttyACM0 115200
```

Went through the Ubuntu setup wizard. See [01-hardware-check.md](01-hardware-check.md)
for the choices made during setup.

---

## Result

- Fresh Ubuntu 18.04.5 / L4T R32.7.1 / JetPack 4.6.1
- Login: `lucia` / `lucia-143-tomato`
- Hostname: `lucia-jetson`
- SSH accessible at `192.168.55.1` (USB device mode)
