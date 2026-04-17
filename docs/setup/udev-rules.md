# udev Rules — Stable Device Names

## Problem

The Raspberry Pi has two USB serial devices: the Roomba 650 and the RPLidar A2.
Linux assigns `/dev/ttyUSB0` and `/dev/ttyUSB1` at boot, but the numbering is not
guaranteed — it depends on which device enumerates first and can change if a device
is unplugged and replugged.

Scripts and ROS2 launch files must reference stable names. udev rules solve this by
creating fixed symlinks (`/dev/roomba`, `/dev/rplidar`) based on the USB device's
unique serial number.

---

## Step 1 — Find each device's serial number

Plug in **only the Roomba** USB-serial cable and run:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|{serial}'
```

Note the `ATTRS{serial}` value. Then unplug it, plug in **only the RPLidar** adapter,
and repeat.

Both devices use CP210x USB-UART chips (idVendor `10c4`, idProduct `ea60`), so the
serial number is what distinguishes them.

---

## Step 2 — Create the rules file

```bash
sudo nano /etc/udev/rules.d/99-lucia.rules
```

```
# Roomba 650 (CP210x)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<roomba_serial>", SYMLINK+="roomba"

# RPLidar A2 (CP2102)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<rplidar_serial>", SYMLINK+="rplidar"
```

Replace `<roomba_serial>` and `<rplidar_serial>` with the values from Step 1.

---

## Step 3 — Reload udev

```bash
sudo udevadm control --reload
sudo udevadm trigger
```

---

## Step 4 — Confirm

With both devices plugged in:

```bash
ls -la /dev/roomba /dev/rplidar
```

Expected output:

```
lrwxrwxrwx 1 root root 7 ... /dev/roomba  -> ttyUSB0
lrwxrwxrwx 1 root root 7 ... /dev/rplidar -> ttyUSB1
```

The symlink targets (`ttyUSB0`, `ttyUSB1`) may vary — that's fine. The symlink names
will always be stable regardless of enumeration order.

---

## Usage in scripts and launch files

| Instead of | Use |
|-----------|-----|
| `/dev/ttyUSB0` | `/dev/roomba` |
| `/dev/ttyUSB1` | `/dev/rplidar` |

The existing `roomba_oi.py` accepts a port argument — pass `/dev/roomba`.
The `rplidar_ros2` launch file accepts `serial_port:=/dev/rplidar`.
