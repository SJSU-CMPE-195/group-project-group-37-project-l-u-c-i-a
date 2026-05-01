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

Default password: `nvidia`

**Actual output:**
```
# paste here
```

---

## Step 2 — Check JetPack version

```bash
cat /etc/nv_tegra_release
```

**Actual output:**
```
# paste here
```

**Interpretation:**
- `R35.x` → JetPack 5.x → Ubuntu 22.04 (Jammy) → ZED SDK 4.x + ROS2 Humble ✓
- `R32.x` → JetPack 4.x → Ubuntu 18.04 (Bionic) → ZED SDK 3.x + ROS2 Foxy (non-ideal)

**Result:** <!-- fill in -->

---

## Step 3 — Confirm ZED 2i is detected

Plug the ZED 2i into the **blue USB 3.0 port** on the Jetson, then:

```bash
lsusb
```

Look for `Stereolabs` or product ID `2b03:f582`.

**Actual output:**
```
# paste here
```

**Result:** <!-- Detected / Not detected -->

---

## Step 4 — Check available disk space

The ZED SDK + AI models + ROS2 Humble + ZED wrapper is roughly 10–15 GB total.

```bash
df -h /
```

**Actual output:**
```
# paste here
```

**Result:** <!-- Enough space / Need to free space -->

---

## Notes

<!-- Any unexpected findings, error messages, or hardware quirks go here -->
