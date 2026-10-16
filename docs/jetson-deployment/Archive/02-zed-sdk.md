# Phase 2 — ZED SDK Installation

Install the ZED SDK 3.x and verify the SDK is present before touching ROS2.

---

## Prerequisites

- JetPack 4.6.1 / L4T R32.7.1 / Ubuntu 18.04 confirmed (Phase 1)
- ZED 2i connected to blue USB 3.0 port
- ~5 GB free disk space
- Internet access on the Jetson

> ⚠️ JetPack 4.x ships CUDA 10.2. ZED SDK 4.x requires CUDA 11+.
> **Must install ZED SDK 3.x** — the last version supporting JetPack 4.6 / CUDA 10.2.

---

## Step 1 — Install zstd (required by installer)

```bash
sudo apt install zstd -y
```

---

## Step 2 — Download the SDK directly on the Jetson

```bash
wget https://download.stereolabs.com/zedsdk/3.8/l4t32.7/jetsons -O ZED_SDK_Jetson_JP46.run
```

Resolves to: `ZED_SDK_Tegra_L4T32.7_v3.8.2.zstd.run` (45 MB, downloads at ~15 MB/s)

---

## Step 3 — Run the installer

```bash
chmod +x ZED_SDK_Jetson_JP46.run
./ZED_SDK_Jetson_JP46.run
```

**Installer prompts — answer as follows:**

| Prompt | Answer |
|--------|--------|
| Accept EULA | Y |
| Install static version | Y |
| Install AI module (object detection) | Y |
| Maximum performance mode | Y |
| Install samples | Y |
| Install Python API | Y → `python3` |
| Run ZED Diagnostic to download AI models | Y |
| Optimize AI models now (takes hours) | Y |

**Warnings encountered:**
- `ERROR: installer failed to detect CUDA version` — CUDA wasn't in PATH yet, fixed in Step 4
- `Python API failed to install` — numpy pip build failed; non-critical for ROS2 use
- `libv4l-dev warning` — do NOT install libv4l-dev, it breaks hardware encode/decode

**Result:** `ZED SDK installation complete, with 1 warning(s)`

---

## Step 4 — Install CUDA toolkit and add to PATH

CUDA was installed by JetPack but nvcc wasn't available. Fix:

```bash
sudo apt install cuda-toolkit-10-2 -y
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
nvcc --version
# nvcc: NVIDIA (R) Cuda compiler driver
# Cuda compilation tools, release 10.2, V10.2.300
```

---

## Step 5 — Verify SDK installation

ZED diagnostic requires a display (no display connected) so verify via filesystem:

```bash
ls /usr/local/zed/tools/
# ZED_Calibration  ZED_Depth_Viewer  ZED_Diagnostic  ZED_Explorer
# ZED_Sensor_Viewer  ZED_SVO_Editor  ZEDfu  jetson_clocks.service

ls /usr/local/zed/lib/
# libsl_ai.so  libsl_zed.so  libsl_zed_static.a
```

**Result:** SDK libraries and tools confirmed present. ✅

---

## Step 6 — Camera confirmed working via C++ sample

Since all ZED tools require a display, compiled the depth sensing C++ sample:

```bash
sudo apt install cmake -y
cd '/usr/local/zed/samples/depth sensing/cpp'
mkdir build && cd build
cmake ..
make -j4
./ZED_Depth_Sensing
```

Output confirmed camera is fully functional:
```
[ZED][INFO] [Init]  Camera successfully opened.
[ZED][INFO] [Init]  Sensors FW version: 776
[ZED][INFO] [Init]  Camera FW version: 1523
[ZED][INFO] [Init]  Video mode: HD720@60
[ZED][INFO] [Init]  Calibration file downloaded.
freeglut: failed to open display ''   ← expected, no display connected
```

**Camera serial number:** 25161264
**Status:** ZED 2i confirmed working ✅

---

## Notes

- ZED SDK version: **3.8.2**
- CUDA: **10.2.300**
- Camera serial number: **25161264**
- Python API install failed (numpy binary incompatibility with Python 3.6) — use C++ API instead
- `ZED_Diagnostic`, `ZED_Explorer`, and Python samples all require a display — test headlessly via C++ sample
- Do NOT install `libv4l-dev` — breaks hardware encode/decode on Jetson
- pyzed can be installed via: `sudo pip3 install /usr/local/zed/pyzed-3.8-cp36-cp36m-linux_aarch64.whl` but numpy version mismatch prevents use
