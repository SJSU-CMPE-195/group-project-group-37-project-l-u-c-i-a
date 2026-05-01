# Phase 2 — ZED SDK Installation

Install the ZED SDK 4.x and verify the camera works before touching ROS2.

---

## Prerequisites

- JetPack version confirmed as R35.x (Phase 1)
- ZED 2i connected to blue USB 3.0 port
- ~5 GB free disk space
- Internet access on the Jetson

---

## Step 1 — Download the SDK installer

On your laptop, go to the Stereolabs website and download the ZED SDK `.run`
file for **JetPack 5.x / L4T**.

Copy it to the Jetson:

```bash
scp ZED_SDK_*.run nvidia@192.168.55.1:~
```

**SDK version downloaded:** <!-- e.g. ZED SDK 4.1.2 for JetPack 5.1 -->

---

## Step 2 — Run the installer

```bash
chmod +x ZED_SDK_*.run
./ZED_SDK_*.run
```

When prompted:
- Accept the license agreement
- **Install AI modules: YES** — required for object detection
- Install samples: optional

Installation takes 10–20 minutes and downloads neural network model files.

**Actual output (summary):**
```
# paste key lines here — version installed, install path, any errors
```

---

## Step 3 — Run the self-test

```bash
/usr/local/zed/tools/ZED_Diagnostic
```

**Actual output:**
```
# paste here
```

**Result:** <!-- Pass / Fail -->

---

## Step 4 — Verify live feed (optional but recommended)

```bash
/usr/local/zed/tools/ZED_Explorer
```

Confirm color image and depth feed are visible with no artifacts.

**Result:** <!-- Working / Issues noted -->

---

## Notes

<!-- SDK version, any install errors, workarounds applied -->
