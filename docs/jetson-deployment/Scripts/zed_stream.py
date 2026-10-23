#!/usr/bin/env python3

import argparse
import threading
import time
import socketserver
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import pyzed.sl as sl


latest_jpeg = None
frame_lock = threading.Lock()
stop_event = threading.Event()


def capture_loop(zed, image):
    global latest_jpeg

    while not stop_event.is_set():
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            time.sleep(0.01)
            continue

        zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()

        if len(frame.shape) == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), 70],
        )

        if success:
            with frame_lock:
                latest_jpeg = encoded.tobytes()


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/stream.mjpg"):
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
            while not stop_event.is_set():
                with frame_lock:
                    jpeg = latest_jpeg

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 15
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE

    error = zed.open(init_params)

    if error != sl.ERROR_CODE.SUCCESS:
        print("Camera open failed:", error)
        return

    image = sl.Mat()

    capture_thread = threading.Thread(
        target=capture_loop,
        args=(zed, image),
    )
    capture_thread.daemon = True
    capture_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), StreamHandler)

    print("Video stream available at:")
    print("http://192.168.10.42:{}/".format(args.port))
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping stream...")
    finally:
        stop_event.set()
        server.server_close()
        capture_thread.join(2)
        zed.close()


if __name__ == "__main__":
    main()

