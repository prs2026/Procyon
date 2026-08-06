#!/usr/bin/env python3
"""Bridge SatDump's rotctld TCP commands to G-code over a serial port.

Example:
    python satellitetracker.py --serial-port COM3 --baud-rate 115200

Each position command is written to both the terminal and serial port:
    G0 X135 Y42

X is azimuth in degrees (0 = north, increasing clockwise), and Y is
elevation in degrees (0 = horizon, 90 = straight up).
"""

from __future__ import annotations

import argparse
import math
import socket
import sys


def validate_position(azimuth: float, elevation: float) -> tuple[float, float]:
    if not math.isfinite(azimuth) or not math.isfinite(elevation):
        raise ValueError("position contains a non-finite number")
    if not 0.0 <= azimuth <= 360.0:
        raise ValueError(f"azimuth {azimuth} is outside 0..360 degrees")
    if not 0.0 <= elevation <= 90.0:
        raise ValueError(f"elevation {elevation} is outside 0..90 degrees")
    return azimuth, elevation


def format_number(value: float) -> str:
    """Keep useful fractional degrees without trailing zeroes."""
    return f"{value:.4f}".rstrip("0").rstrip(".")


def position_to_gcode(azimuth: float, elevation: float) -> str:
    # SatDump azimuth already uses north=0 and increases clockwise.
    return f"G0 X{format_number(azimuth)} Y{format_number(elevation)}"


def parse_set_position(command: str) -> tuple[float, float]:
    """Parse rotctld's short ``P azimuth elevation`` command."""
    fields = command.split()
    if len(fields) != 3 or fields[0] != "P":
        raise ValueError("expected 'P <azimuth> <elevation>'")
    return validate_position(float(fields[1]), float(fields[2]))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bind",
        default="127.0.0.1",
        help="local address on which to listen (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=4533,
        help="TCP port on which to listen (default: 4533)",
    )
    parser.add_argument(
        "--serial-port",
        required=True,
        help="serial device receiving G-code, such as COM3 or /dev/ttyUSB0",
    )
    parser.add_argument(
        "--baud-rate",
        type=int,
        default=115200,
        help="serial baud rate (default: 115200)",
    )
    return parser.parse_args()


def open_serial(port: str, baud_rate: int):
    try:
        import serial
    except ImportError as error:
        raise RuntimeError(
            "pyserial is required; install it with: python -m pip install pyserial"
        ) from error

    try:
        return serial.Serial(port=port, baudrate=baud_rate, timeout=1)
    except serial.SerialException as error:
        raise RuntimeError(f"could not open serial port {port!r}: {error}") from error


def send_gcode(serial_port, azimuth: float, elevation: float) -> None:
    gcode = position_to_gcode(azimuth, elevation)
    serial_port.write((gcode + "\n").encode("ascii"))
    serial_port.flush()
    print(gcode, flush=True)


def handle_client(client: socket.socket, address, serial_port) -> None:
    """Serve one SatDump connection until it disconnects."""
    last_position = (0.0, 0.0)
    pending = b""
    print(f"SatDump connected from {address[0]}:{address[1]}", file=sys.stderr)

    while True:
        data = client.recv(4096)
        if not data:
            break
        pending += data

        while b"\n" in pending:
            raw_command, pending = pending.split(b"\n", 1)
            command = raw_command.decode("ascii").strip().strip("\r")
            if not command:
                continue

            if command == "p":
                azimuth, elevation = last_position
                response = (
                    f"{format_number(azimuth)}\n"
                    f"{format_number(elevation)}\n"
                )
                client.sendall(response.encode("ascii"))
                continue

            try:
                position = parse_set_position(command)
                send_gcode(serial_port, *position)
                last_position = position
                client.sendall(b"RPRT 0\n")
            except (OverflowError, UnicodeDecodeError, ValueError) as error:
                print(f"Ignoring TCP command {command!r}: {error}", file=sys.stderr)
                client.sendall(b"RPRT -1\n")

    print(f"SatDump disconnected from {address[0]}:{address[1]}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    if not 1 <= args.port <= 65535:
        print("error: --port must be between 1 and 65535", file=sys.stderr)
        return 2
    if args.baud_rate <= 0:
        print("error: --baud-rate must be positive", file=sys.stderr)
        return 2

    try:
        serial_port = open_serial(args.serial_port, args.baud_rate)
    except RuntimeError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    with serial_port, socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.bind, args.port))
        server.listen()
        print(
            f"Listening for SatDump on tcp://{args.bind}:{args.port}; "
            f"sending G-code to {args.serial_port} at {args.baud_rate} baud",
            file=sys.stderr,
        )

        try:
            while True:
                client, address = server.accept()
                with client:
                    try:
                        handle_client(client, address, serial_port)
                    except (ConnectionError, OSError) as error:
                        print(f"SatDump connection error: {error}", file=sys.stderr)
        except KeyboardInterrupt:
            print("\nStopped.", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
