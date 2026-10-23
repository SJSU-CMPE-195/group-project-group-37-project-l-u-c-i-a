#!/usr/bin/env python3
"""Minimal standard-library UDP listener for follow_sender.py."""

import argparse
import json
import socket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    print("Listening on {}:{}".format(args.bind, args.port))

    try:
        while True:
            payload, address = sock.recvfrom(65535)
            try:
                message = json.loads(payload.decode("utf-8"))
                print("{}:{} {}".format(
                    address[0], address[1],
                    json.dumps(message, separators=(",", ":")),
                ))
            except (UnicodeDecodeError, ValueError) as error:
                print("{}:{} invalid packet: {}".format(address[0], address[1], error))
    except KeyboardInterrupt:
        print("\nListener stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
