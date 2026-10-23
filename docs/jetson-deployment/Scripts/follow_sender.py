#!/usr/bin/env python3
"""ZED 2 body/sign tracker with UDP detection and HTTP video streaming."""

import argparse
import json
import math
import socket
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer


SEND_INTERVAL_SECONDS = 0.10
SIGN_HOLD_SECONDS = 1.0
SIGN_COOLDOWN_SECONDS = 2.0
SIGN_REPEAT_MESSAGES = 4
SIGN_CLEAR_MARGIN_PIXELS = 12.0

NOSE = 0
RIGHT_WRIST = 4
LEFT_WRIST = 7
RIGHT_EYE = 14
LEFT_EYE = 15

SKELETON_BONES = (
    (0, 1),
    (1, 2), (2, 3), (3, 4),
    (1, 5), (5, 6), (6, 7),
    (1, 8), (8, 9), (9, 10),
    (1, 11), (11, 12), (12, 13),
    (0, 14), (14, 16),
    (0, 15), (15, 17),
)

LATEST_VIDEO_JPEG = None
VIDEO_FRAME_LOCK = threading.Lock()
VIDEO_STOP = threading.Event()


def _update_video_frame(cv2, frame):
    global LATEST_VIDEO_JPEG

    success, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), 70],
    )

    if success:
        with VIDEO_FRAME_LOCK:
            LATEST_VIDEO_JPEG = encoded.tobytes()


class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?", 1)[0]

        if path not in ("/", "/stream.mjpg"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()

        try:
            while not VIDEO_STOP.is_set():
                with VIDEO_FRAME_LOCK:
                    jpeg = LATEST_VIDEO_JPEG

                if jpeg is None:
                    time.sleep(0.05)
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    ("Content-Length: {}\r\n\r\n".format(len(jpeg))).encode()
                )
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()

                time.sleep(0.1)

        except OSError:
            pass

    def log_message(self, format_string, *args):
        pass


class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def _xy(point):
    try:
        x = float(point[0])
        y = float(point[1])
    except (IndexError, KeyError, TypeError, ValueError):
        try:
            x = float(point.x)
            y = float(point.y)
        except (AttributeError, TypeError, ValueError):
            return None

    if not math.isfinite(x) or not math.isfinite(y):
        return None

    return x, y


def classify_arm_sign(
    keypoints,
    clear_margin_pixels=SIGN_CLEAR_MARGIN_PIXELS,
):
    if keypoints is None:
        return None

    try:
        head_points = [
            _xy(keypoints[index])
            for index in (NOSE, RIGHT_EYE, LEFT_EYE)
        ]
        right_wrist = _xy(keypoints[RIGHT_WRIST])
        left_wrist = _xy(keypoints[LEFT_WRIST])
    except (IndexError, KeyError, TypeError):
        return None

    valid_head_points = [
        point for point in head_points if point is not None
    ]

    if not valid_head_points:
        return None

    if right_wrist is None or left_wrist is None:
        return None

    head_y = min(point[1] for point in valid_head_points)

    right_up = right_wrist[1] < head_y - clear_margin_pixels
    left_up = left_wrist[1] < head_y - clear_margin_pixels

    if right_up and left_up:
        return "stop"

    if right_up != left_up:
        return "follow"

    return None


class SignDetector(object):
    def __init__(
        self,
        hold_seconds=SIGN_HOLD_SECONDS,
        cooldown_seconds=SIGN_COOLDOWN_SECONDS,
        clear_margin_pixels=SIGN_CLEAR_MARGIN_PIXELS,
    ):
        self.hold_seconds = float(hold_seconds)
        self.cooldown_seconds = float(cooldown_seconds)
        self.clear_margin_pixels = float(clear_margin_pixels)
        self._states = {}

    def update(self, person_id, keypoints, timestamp):
        now = float(timestamp)

        kind = classify_arm_sign(
            keypoints,
            self.clear_margin_pixels,
        )

        state = self._states.setdefault(
            int(person_id),
            {
                "candidate": None,
                "started": None,
                "cooldown_until": 0.0,
            },
        )

        if now < state["cooldown_until"]:
            return None

        if kind is None:
            state["candidate"] = None
            state["started"] = None
            return None

        if state["candidate"] != kind:
            state["candidate"] = kind
            state["started"] = now
            return None

        if now - state["started"] >= self.hold_seconds:
            state["candidate"] = None
            state["started"] = None
            state["cooldown_until"] = now + self.cooldown_seconds
            return kind

        return None


def _test_keypoints(right_wrist_y, left_wrist_y):
    points = [(100.0, 300.0)] * 18

    points[NOSE] = (100.0, 200.0)
    points[RIGHT_EYE] = (95.0, 190.0)
    points[LEFT_EYE] = (105.0, 190.0)
    points[RIGHT_WRIST] = (80.0, right_wrist_y)
    points[LEFT_WRIST] = (120.0, left_wrist_y)

    return points


def run_self_test():
    detector = SignDetector()

    follow_pose = _test_keypoints(120.0, 250.0)

    assert detector.update(3, follow_pose, 0.0) is None
    assert detector.update(3, follow_pose, 0.9) is None
    assert detector.update(3, follow_pose, 1.01) == "follow"
    assert detector.update(3, follow_pose, 1.10) is None

    stop_detector = SignDetector()
    stop_pose = _test_keypoints(120.0, 120.0)

    assert stop_detector.update(8, stop_pose, 10.0) is None
    assert stop_detector.update(8, stop_pose, 11.01) == "stop"

    invalid_detector = SignDetector()
    invalid_pose = _test_keypoints(float("nan"), 120.0)

    assert invalid_detector.update(9, invalid_pose, 20.0) is None

    print("Sign logic self-test passed.")


class UdpSender(object):
    def __init__(self, host, port):
        self.destination = (host, int(port))
        self.socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

    def send(self, message):
        payload = json.dumps(
            message,
            separators=(",", ":"),
        ).encode("utf-8")

        try:
            self.socket.sendto(payload, self.destination)
        except OSError:
            pass

    def close(self):
        self.socket.close()


def _tracking_is_ok(sl, obj):
    state = getattr(obj, "tracking_state", None)
    enum_type = getattr(sl, "OBJECT_TRACKING_STATE", None)
    expected = getattr(enum_type, "OK", None)

    if expected is not None:
        try:
            return state == expected
        except Exception:
            pass

    return "OK" in str(state).upper()


def _valid_position(position):
    try:
        values = (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )
    except (IndexError, KeyError, TypeError, ValueError):
        return None

    if not all(math.isfinite(value) for value in values):
        return None

    return values


def _person_record(person_id, position, confidence):
    return {
        "id": int(person_id),
        "x": round(position[0], 2),
        "y": round(position[1], 2),
        "z": round(position[2], 2),
        "conf": round(float(confidence), 2),
    }


def _collect_objects(sl, objects, sign_detector, timestamp):
    people = []
    fired_sign = None

    for obj in getattr(objects, "object_list", []):
        if not _tracking_is_ok(sl, obj):
            continue

        person_id = int(obj.id)
        keypoints = getattr(obj, "keypoint_2d", None)

        sign_kind = sign_detector.update(
            person_id,
            keypoints,
            timestamp,
        )

        if sign_kind is not None:
            fired_sign = {
                "id": person_id,
                "kind": sign_kind,
            }

        position = _valid_position(
            getattr(obj, "position", None)
        )

        if position is None:
            continue

        people.append(
            _person_record(
                person_id,
                position,
                obj.confidence,
            )
        )

    people.sort(key=lambda item: item["id"])

    return people, fired_sign


def _draw_overlay(cv2, frame, sl, objects, sign_text):
    for obj in getattr(objects, "object_list", []):
        if not _tracking_is_ok(sl, obj):
            continue

        keypoints = getattr(obj, "keypoint_2d", None)
        points = {}

        if keypoints is not None:
            for index in range(18):
                try:
                    point = _xy(keypoints[index])
                except (IndexError, KeyError, TypeError):
                    point = None

                if point is not None:
                    points[index] = (
                        int(point[0]),
                        int(point[1]),
                    )

            for first, second in SKELETON_BONES:
                if first in points and second in points:
                    cv2.line(
                        frame,
                        points[first],
                        points[second],
                        (255, 180, 0),
                        2,
                    )

            for point in points.values():
                cv2.circle(
                    frame,
                    point,
                    4,
                    (0, 255, 255),
                    -1,
                )

        position = _valid_position(
            getattr(obj, "position", None)
        )

        if position is not None:
            distance = math.sqrt(
                sum(value * value for value in position)
            )
            label = "ID {}  {:.2f} m".format(
                int(obj.id),
                distance,
            )
        else:
            label = "ID {}  position unavailable".format(
                int(obj.id)
            )

        label_point = points.get(NOSE, (10, 60))

        cv2.putText(
            frame,
            label,
            (
                label_point[0],
                max(label_point[1] - 12, 20),
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
        )

    cv2.putText(
        frame,
        "sign: {}".format(sign_text),
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 200, 255),
        2,
    )


def _message(sequence, people, sign):
    return {
        "seq": int(sequence),
        "people": people,
        "sign": sign,
    }


def run_sender(args):
    try:
        import cv2
        import pyzed.sl as sl
    except ImportError as error:
        raise RuntimeError(
            "This mode must run on the Nano with pyzed and "
            "OpenCV installed: {}".format(error)
        )

    zed = sl.Camera()
    sender = UdpSender(args.host, args.port)
    sign_detector = SignDetector()

    positional_tracking_enabled = False
    object_detection_enabled = False
    stream_server = None
    stream_thread = None

    last_sign_text = "none"
    last_sign_display_until = 0.0
    repeat_sign = None
    repeat_messages_left = 0
    sequence = 0

    try:
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = int(args.fps)
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = (
            sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        )

        error_code = zed.open(init_params)

        if error_code != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                "Camera open failed: {}".format(error_code)
            )

        runtime = sl.RuntimeParameters()
        runtime.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA

        body_params = sl.ObjectDetectionParameters()
        body_params.detection_model = (
            sl.DETECTION_MODEL.HUMAN_BODY_FAST
        )
        body_params.body_format = sl.BODY_FORMAT.POSE_18
        body_params.enable_tracking = True
        body_params.enable_body_fitting = True

        positional_tracking_params = (
            sl.PositionalTrackingParameters()
        )

        error_code = zed.enable_positional_tracking(
            positional_tracking_params
        )

        if error_code != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                "Positional tracking failed: {}".format(error_code)
            )

        positional_tracking_enabled = True

        print(
            "Body model may optimize on first run; "
            "this can take several minutes."
        )

        error_code = zed.enable_object_detection(body_params)

        if error_code != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(
                "Body/object detection failed: {}".format(error_code)
            )

        object_detection_enabled = True

        if args.stream:
            VIDEO_STOP.clear()

            stream_server = ThreadingHTTPServer(
                ("0.0.0.0", args.stream_port),
                VideoStreamHandler,
            )

            stream_thread = threading.Thread(
                target=stream_server.serve_forever
            )
            stream_thread.daemon = True
            stream_thread.start()

            print(
                "Video stream available at "
                "http://<NANO_IP>:{}/".format(
                    args.stream_port
                )
            )

        objects = sl.Objects()
        object_runtime = sl.ObjectDetectionRuntimeParameters()
        object_runtime.detection_confidence_threshold = 40

        image = sl.Mat() if (args.show or args.stream) else None
        next_send = time.monotonic()

        while True:
            now = time.monotonic()
            current_people = []
            current_objects = None

            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_objects(
                    objects,
                    object_runtime,
                )

                current_objects = objects

                current_people, fired_sign = _collect_objects(
                    sl,
                    objects,
                    sign_detector,
                    now,
                )

                if fired_sign is not None:
                    repeat_sign = fired_sign
                    repeat_messages_left = SIGN_REPEAT_MESSAGES

                    last_sign_text = "{} ({})".format(
                        fired_sign["kind"],
                        fired_sign["id"],
                    )

                    last_sign_display_until = (
                        now + SIGN_COOLDOWN_SECONDS
                    )

                if args.show or args.stream:
                    zed.retrieve_image(
                        image,
                        sl.VIEW.LEFT,
                        sl.MEM.CPU,
                    )

                    frame = image.get_data()

                    if (
                        len(frame.shape) == 3
                        and frame.shape[2] == 4
                    ):
                        frame = cv2.cvtColor(
                            frame,
                            cv2.COLOR_RGBA2BGR,
                        )
                    else:
                        frame = frame.copy()

                    if now >= last_sign_display_until:
                        last_sign_text = "none"

                    _draw_overlay(
                        cv2,
                        frame,
                        sl,
                        current_objects,
                        last_sign_text,
                    )

                    if args.stream:
                        _update_video_frame(
                            cv2,
                            frame,
                        )

                    if args.show:
                        cv2.imshow(
                            "ZED 2 Follow Sender",
                            frame,
                        )

                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break

            else:
                current_people = []

            now = time.monotonic()

            if now >= next_send:
                sign_for_packet = (
                    repeat_sign
                    if repeat_messages_left > 0
                    else None
                )

                packet = _message(
                    sequence,
                    current_people,
                    sign_for_packet,
                )

                sender.send(packet)

                if args.print_messages:
                    print(
                        json.dumps(
                            packet,
                            separators=(",", ":"),
                        )
                    )

                sequence += 1

                if repeat_messages_left > 0:
                    repeat_messages_left -= 1

                    if repeat_messages_left == 0:
                        repeat_sign = None

                next_send = now + SEND_INTERVAL_SECONDS

    finally:
        if stream_server is not None:
            VIDEO_STOP.set()

            if stream_thread is not None:
                stream_server.shutdown()

            stream_server.server_close()

            if stream_thread is not None:
                stream_thread.join(2)

        sender.close()

        if object_detection_enabled:
            zed.disable_object_detection()

        if positional_tracking_enabled:
            zed.disable_positional_tracking()

        zed.close()

        if args.show:
            cv2.destroyAllWindows()


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        "--host",
        default="192.168.50.1",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=5005,
    )

    parser.add_argument(
        "--fps",
        type=int,
        choices=(15, 30),
        default=15,
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="show the optional local OpenCV overlay",
    )

    parser.add_argument(
        "--print",
        dest="print_messages",
        action="store_true",
        help="print each JSON packet",
    )

    parser.add_argument(
        "--stream",
        action="store_true",
        help="serve the annotated video over HTTP",
    )

    parser.add_argument(
        "--stream-port",
        type=int,
        default=8080,
        help="TCP port used for the video stream",
    )

    parser.add_argument(
        "--self-test",
        action="store_true",
        help="test sign logic without opening the camera",
    )

    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(
        sys.argv[1:] if argv is None else argv
    )

    if args.self_test:
        run_self_test()
        return 0

    try:
        run_sender(args)

    except KeyboardInterrupt:
        print("\nStopping follow sender.")

    except RuntimeError as error:
        print(
            "Error: {}".format(error),
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
