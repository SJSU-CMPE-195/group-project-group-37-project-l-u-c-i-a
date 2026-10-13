# Jetson Nano and ZED 2 Setup Notes

## Project goal

Use the Jetson Nano to capture data from a Stereolabs ZED 2 camera, perform object detection and depth processing on the Nano, and later send useful results to a Raspberry Pi. A video stream may also be sent to the Pi for reference or debugging.

Planned data flow:

```text
ZED 2 camera → Jetson Nano → network → Raspberry Pi
                    ├─ object detection and depth processing
                    └─ optional video/reference stream
```

## Hardware and software identified

| Item | Result |
|---|---|
| Jetson board | Official Jetson Nano module with onboard eMMC; no microSD slot |
| Carrier board | Waveshare `JETSON-IO-BASE-A` |
| Jetson Linux | `R32.7.6` (`aarch64`), JetPack 4.6-era software |
| Python | Python 3.6 |
| ZED camera | ZED 2 |
| ZED SDK | 3.8.2, Python API successfully imported |
| OpenCV | 3.2.0, Python module successfully imported |
| Camera device | `/dev/video0` |

## Jetson login and initial camera check

The Nano was already booting into Ubuntu and eventually allowed login with the existing account. The camera was connected to a USB 3.0 port.

The ZED tools were located with:

```bash
ls /usr/local/zed/tools
```

The live camera feed was successfully opened with:

```bash
/usr/local/zed/tools/ZED_Explorer
```

Linux filenames are case-sensitive. `Zed_Explorer` failed, while `ZED_Explorer` worked.

The camera was also confirmed at the Linux video-device level:

```bash
ls -l /dev/video*
```

Result included:

```text
/dev/video0
```

The Jetson software version was checked with:

```bash
cat /etc/nv_tegra_release
```

The relevant result was:

```text
# R32 (release), REVISION: 7.6, BOARD: t210ref, EABI: aarch64
```

## Repairing the Python dependencies

The first ZED Python test failed because the system was missing the BLAS library used by NumPy:

```text
libblas.so.3: cannot open shared object file
```

After the Nano was connected to the internet, the required Ubuntu packages were installed:

```bash
sudo apt update
sudo apt install --reinstall libblas3 liblapack3 python3-numpy
```

The first `apt update` attempt failed because the Nano could not resolve the repository addresses. After internet access was connected, the packages installed successfully.

## ZED Python API compatibility fix

The initial Python import then produced:

```text
ValueError: numpy.ufunc size changed, may indicate binary incompatibility
```

This meant that the installed ZED Python wrapper and NumPy were built against incompatible NumPy interfaces.

The official ZED Python installer was attempted, but on this older Python 3.6/ARM64 system it tried to build NumPy 1.19.5 from source using a newer Cython version. That build failed. A NumPy 1.19.5 ARM64 wheel also caused an illegal-instruction crash on the Nano.

The working solution was the Jetson Nano-specific NumPy 1.19.4 wheel:

```bash
python3 -m pip install --user --force-reinstall --no-deps \
"https://github.com/jetson-nano-wheels/python3.6-numpy-1.19.4/releases/download/v0.0.2/numpy-1.19.4-cp36-cp36m-linux_aarch64.whl"
```

The downloaded ZED Python wheel was then installed without changing its dependencies:

```bash
python3 -m pip install --user --no-deps --force-reinstall \
/home/lucia/pyzed-3.8-cp36-cp36m-linux_aarch64.whl
```

The final version test succeeded:

```bash
python3 -c "import numpy; print(numpy.__version__); import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"
```

Successful output:

```text
1.19.4
3.8.2
```

## OpenCV installation

Python initially reported:

```text
No module named cv2
```

OpenCV was installed using Ubuntu's package manager:

```bash
sudo apt update
sudo apt install python3-opencv
```

The Python OpenCV test succeeded:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

Result:

```text
3.2.0
```

## Camera-open test

The ZED 2 was successfully opened through the Python API with:

```bash
python3 -c "import pyzed.sl as sl; z=sl.Camera(); p=sl.InitParameters(); e=z.open(p); print(e); z.close()"
```

Result:

```text
SUCCESS
```

This confirms that the ZED 2 can be accessed from a Python program, not just from ZED Explorer.

## Camera compatibility

The current combination is compatible for the planned project:

| Capability | Status |
|---|---|
| Live video from ZED 2 | Confirmed working in ZED Explorer |
| USB connection | Confirmed; camera appears as `/dev/video0` |
| ZED Python API | Confirmed working with SDK 3.8.2 |
| Python/OpenCV display | Dependencies confirmed; detection display program not yet run |
| ZED 2 object detection | Supported by the ZED SDK |
| Depth and 3D position | Supported by the ZED 2 and SDK |
| Raspberry Pi communication | Not started; Pi is not currently available |

The original ZED 1 is not compatible with the ZED SDK object-detection module. The identified camera is a ZED 2, so it is suitable for object detection, depth, 3D position, and tracking.

Because the Nano is running JetPack 4.6-era software, the ZED SDK must remain matched to the Nano's Jetson Linux/L4T version. Avoid installing a modern SDK blindly over SDK 3.8.2.

## Current status

Completed:

- Nano boots into Ubuntu.
- Existing Ubuntu account can be used.
- Nano has internet access.
- ZED 2 live feed works.
- ZED SDK 3.8.2 Python API works.
- NumPy 1.19.4 works on the Nano.
- OpenCV 3.2.0 is installed.
- Python can open and close the ZED 2 successfully.

Paused:

- The Python object-detection/display program has not yet been created.
- Nano-to-Raspberry-Pi networking has not yet been configured.
- The Windows dual-boot and partition work is paused.

## Next planned steps

1. Run a low-resolution ZED 2 object-detection program using the fast detection model and display the annotated video locally on the Nano.
2. Send lightweight detection messages—label, confidence, distance, and position—from the Nano to the Raspberry Pi.
3. Add an optional compressed video stream for reference.
4. Return to the Windows installation USB and repair the Windows NTFS partition before attempting to resize it for dual boot.

## Useful references

- [Stereolabs ZED SDK documentation](https://docs.stereolabs.com/)
- [Stereolabs ZED 2 object-detection tutorial](https://github.com/stereolabs/zed-sdk/tree/master/tutorials/tutorial%206%20-%20object%20detection/python)
- [Jetson Nano NumPy 1.19.4 wheel](https://github.com/jetson-nano-wheels/python3.6-numpy-1.19.4)


## Notes
Updated Password from `lucia-143-tomato` -> `lucia-tomato`
    This was due to issues of password not working at home setup, but working within school computers
        May be due to compatibility issues?