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

## Power and hardware stability

Connecting the ZED 2 initially caused the Nano to power off. This was consistent with the Nano's power supply being unable to handle the camera and the other connected USB devices together.

After switching to a stronger regulated power adapter, the Nano remained stable while the ZED 2 was connected and the detection program was running.

Recommended arrangement:

- Connect the ZED 2 directly to a USB 3.0 port.
- Avoid putting the ZED 2 behind a USB hub.
- Use the stronger power adapter whenever the camera is connected.

## cuDNN, TensorRT, and AI model setup

Object detection initially failed because the Nano was missing required AI acceleration components. The following TensorRT packages were installed for the JetPack 4.6/L4T R32.7 environment:

```bash
sudo apt install libnvinfer8 libnvinfer-plugin8 libnvonnxparsers8
sudo apt install libnvparsers8 libnvinfer-bin
sudo apt install libnvinfer-dev libnvinfer-plugin-dev \
libnvparsers-dev libnvonnxparsers-dev
```

The TensorRT command-line tool was verified successfully:

```bash
/usr/src/tensorrt/bin/trtexec --help
```

Successful output identified:

```text
TensorRT v8.2.1
```

The ZED AI library dependencies were also checked. TensorRT, its plugin and ONNX parser libraries, and cuDNN all resolved successfully through `ldd` on:

```text
/usr/local/zed/lib/libsl_ai.so
```

The ZED object-detection model was then downloaded and optimized with:

```bash
sudo /usr/local/zed/tools/ZED_Diagnostic -c -ais 0
```

The output reached:

```text
Optimizing: MULTI CLASS DETECTION...
Optimized model not found, trying to download it...
```

This completed the required first-time model preparation for the fast multi-class detection model.

## ZED SDK 3.8.2 object-detection API compatibility

The first detection script used API names from a newer ZED SDK and required several compatibility changes for SDK 3.8.2:

- `enable_segmentation` was not available in `ObjectDetectionParameters`; it was removed.
- `OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST` was not available; the SDK 3.8-compatible detection model enum was used instead:

```python
sl.DETECTION_MODEL.MULTI_CLASS_BOX
```

- `Camera.set_object_detection_runtime_parameters()` was not available in this Python API version. Runtime parameters must be passed directly when retrieving objects:

```python
zed.retrieve_objects(objects, object_runtime)
```

After these changes, the object-detection script ran successfully.

## Working object-detection code

The following is the SDK 3.8.2-compatible version of `zed_detect.py` reconstructed from the working detector setup. It displays the ZED 2 video feed, draws 2D detection boxes, prints detected objects, and exits when `q` is pressed.

```python
import cv2
import pyzed.sl as sl


zed = sl.Camera()

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 30
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE

err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Camera open failed:", err)
    raise SystemExit(1)

object_params = sl.ObjectDetectionParameters()
object_params.enable_tracking = False
object_params.detection_model = sl.DETECTION_MODEL.MULTI_CLASS_BOX

err = zed.enable_object_detection(object_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Object detection failed:", err)
    zed.close()
    raise SystemExit(1)

objects = sl.Objects()
object_runtime = sl.ObjectDetectionRuntimeParameters()
object_runtime.detection_confidence_threshold = 40

image = sl.Mat()

try:
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # SDK 3.8.2 takes runtime parameters here, during retrieval.
            zed.retrieve_objects(objects, object_runtime)
            zed.retrieve_image(image, sl.VIEW.LEFT)

            frame = image.get_data()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

            if objects.is_new:
                for obj in objects.object_list:
                    bbox = obj.bounding_box_2d
                    x1 = int(bbox[0][0])
                    y1 = int(bbox[0][1])
                    x2 = int(bbox[2][0])
                    y2 = int(bbox[2][1])

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    label = "{} {:.1f}%".format(obj.label, obj.confidence)
                    cv2.putText(
                        frame,
                        label,
                        (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

                    print(
                        "{} | confidence: {:.1f}% | position: {}".format(
                            obj.label, obj.confidence, obj.position
                        )
                    )

            cv2.imshow("ZED 2 Object Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
finally:
    zed.disable_object_detection()
    zed.close()
    cv2.destroyAllWindows()
```

To keep a backup before making future edits:

```bash
cp zed_detect.py zed_detect_working.py
```

For this Nano's SDK version, keep these compatibility details in place:

- Use `sl.DETECTION_MODEL.MULTI_CLASS_BOX`.
- Do not use `enable_segmentation`.
- Do not call `set_object_detection_runtime_parameters`; pass `object_runtime` to `zed.retrieve_objects(...)`.

## Camera compatibility

The current combination is compatible for the planned project:

| Capability | Status |
|---|---|
| Live video from ZED 2 | Confirmed working in ZED Explorer |
| USB connection | Confirmed; camera appears as `/dev/video0` |
| ZED Python API | Confirmed working with SDK 3.8.2 |
| Python/OpenCV display | Confirmed working with the detection program |
| ZED 2 object detection | Confirmed working after TensorRT/cuDNN setup and model optimization |
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
- The stronger power adapter prevents shutdown when the ZED 2 is connected.
- TensorRT 8.2.1 and the required cuDNN/TensorRT libraries are available.
- The ZED multi-class object-detection model has been downloaded and optimized.
- The Python object-detection script runs successfully with the SDK 3.8.2-compatible API.

Paused:

- Sending object-detection data to the Raspberry Pi has not yet been configured.
- Optional video streaming to the Raspberry Pi has not yet been configured.
- Nano-to-Raspberry-Pi networking has not yet been configured.
- The Windows dual-boot and partition work is paused.

## Next planned steps

1. Add or verify on-screen labels, confidence values, and 3D positions for each detected object.
2. Save and document the working object-detection script.
3. Send lightweight detection messages—label, confidence, distance, and position—from the Nano to the Raspberry Pi.
4. Add an optional compressed video stream for reference.
5. Return to the Windows installation USB and repair the Windows NTFS partition before attempting to resize it for dual boot.

## Useful references

- [Stereolabs ZED SDK documentation](https://docs.stereolabs.com/)
- [Stereolabs ZED 2 object-detection tutorial](https://github.com/stereolabs/zed-sdk/tree/master/tutorials/tutorial%206%20-%20object%20detection/python)
- [Jetson Nano NumPy 1.19.4 wheel](https://github.com/jetson-nano-wheels/python3.6-numpy-1.19.4)


## Personal notes
- Changed the Jetson's password (the old and new ones are kept off GitHub)
    * This was due to issues of password not working at home setup, but working within school computers
        * May be due to compatibility issues?
    * Issue solved: Password and compatibility are ok, issue was keyboard itself
 
- Make sure to use a power adapter with at least 5V 4A or Nano will turn off 
