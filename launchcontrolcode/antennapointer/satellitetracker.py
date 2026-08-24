#!/usr/bin/env python3
"""Tkinter bridge from SatDump's rotctld TCP commands to serial G-code.

SatDump connects to the TCP listener in this application. Each valid position
command is displayed in the GUI, printed to the terminal, and sent to the
selected serial port as::

    G0 X135 Y42

X is azimuth in degrees (0 = north, increasing clockwise), and Y is elevation
in degrees (0 = horizon, 90 = straight up).
"""

from __future__ import annotations

import math
import queue
import socket
import sys
import threading
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk


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


def format_signed_number(value: float) -> str:
    sign = "+" if value >= 0 else "-"
    return sign + format_number(abs(value))


def position_to_gcode(
    azimuth: float,
    elevation: float,
    x_trim: float = 0.0,
    y_trim: float = 0.0,
) -> str:
    # SatDump azimuth already uses north=0 and increases clockwise.
    return (
        f"G0 X{format_number(azimuth + x_trim)} "
        f"Y{format_number(elevation + y_trim)}"
    )


def parse_set_position(command: str) -> tuple[float, float]:
    """Parse rotctld's short ``P azimuth elevation`` command."""
    fields = command.split()
    if len(fields) != 3 or fields[0] != "P":
        raise ValueError("expected 'P <azimuth> <elevation>'")
    return validate_position(float(fields[1]), float(fields[2]))


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


def available_serial_ports() -> list[str]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [port.device for port in list_ports.comports()]


class SatelliteTrackerGUI:
    POLL_INTERVAL_MS = 100

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("SatDump G-code Bridge")
        self.root.minsize(680, 650)

        self.bind_address = tk.StringVar(value="127.0.0.1")
        self.tcp_port = tk.StringVar(value="4533")
        self.serial_port_name = tk.StringVar()
        self.baud_rate = tk.StringVar(value="115200")
        self.satdump_status = tk.StringVar(value="Disconnected")
        self.serial_status = tk.StringVar(value="Disconnected")
        self.last_gcode = tk.StringVar(value="—")
        self.x_trim_step = tk.StringVar(value="1.0")
        self.y_trim_step = tk.StringVar(value="1.0")
        self.trim_status = tk.StringVar(value="X +0 degrees, Y +0 degrees")
        self.manual_command = tk.StringVar()
        self.manual_status = tk.StringVar(value="Connect serial port to send")

        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.trim_lock = threading.Lock()
        self.x_trim = 0.0
        self.y_trim = 0.0
        self.server_socket: socket.socket | None = None
        self.client_socket: socket.socket | None = None
        self.server_stop = threading.Event()
        self.server_thread: threading.Thread | None = None
        self.listener_description = ""
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.last_position = (0.0, 0.0)
        self.tcp_client_active = False

        self._build_interface()
        self.refresh_serial_ports(show_error=False)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(self.POLL_INTERVAL_MS, self._process_events)

    def _build_interface(self) -> None:
        container = ttk.Frame(self.root, padding=12)
        container.pack(fill="both", expand=True)
        container.columnconfigure(0, weight=1)

        tcp_frame = ttk.LabelFrame(container, text="SatDump TCP", padding=10)
        tcp_frame.grid(row=0, column=0, sticky="ew")
        tcp_frame.columnconfigure(1, weight=1)

        ttk.Label(tcp_frame, text="Bind address:").grid(row=0, column=0, sticky="w")
        self.bind_entry = ttk.Entry(tcp_frame, textvariable=self.bind_address)
        self.bind_entry.grid(row=0, column=1, sticky="ew", padx=(8, 12))
        ttk.Label(tcp_frame, text="Port:").grid(row=0, column=2, sticky="w")
        self.tcp_port_entry = ttk.Entry(tcp_frame, textvariable=self.tcp_port, width=8)
        self.tcp_port_entry.grid(row=0, column=3, sticky="w", padx=(8, 0))

        self.satdump_connect_button = ttk.Button(
            tcp_frame, text="Connect", command=self.connect_satdump
        )
        self.satdump_connect_button.grid(row=1, column=0, pady=(10, 0), sticky="ew")
        self.satdump_disconnect_button = ttk.Button(
            tcp_frame,
            text="Disconnect",
            command=self.disconnect_satdump,
            state="disabled",
        )
        self.satdump_disconnect_button.grid(
            row=1, column=1, pady=(10, 0), padx=(8, 12), sticky="w"
        )
        ttk.Label(tcp_frame, textvariable=self.satdump_status).grid(
            row=1, column=2, columnspan=2, pady=(10, 0), sticky="w"
        )

        serial_frame = ttk.LabelFrame(container, text="G-code Serial Output", padding=10)
        serial_frame.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        serial_frame.columnconfigure(1, weight=1)

        ttk.Label(serial_frame, text="Serial port:").grid(row=0, column=0, sticky="w")
        self.serial_port_combo = ttk.Combobox(
            serial_frame, textvariable=self.serial_port_name
        )
        self.serial_port_combo.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        self.refresh_button = ttk.Button(
            serial_frame, text="Refresh", command=self.refresh_serial_ports
        )
        self.refresh_button.grid(row=0, column=2, sticky="w")
        ttk.Label(serial_frame, text="Baud:").grid(
            row=0, column=3, sticky="w", padx=(12, 0)
        )
        self.baud_combo = ttk.Combobox(
            serial_frame,
            textvariable=self.baud_rate,
            values=("9600", "19200", "38400", "57600", "115200", "230400"),
            width=10,
        )
        self.baud_combo.grid(row=0, column=4, sticky="w", padx=(8, 0))

        self.serial_connect_button = ttk.Button(
            serial_frame, text="Connect", command=self.connect_serial
        )
        self.serial_connect_button.grid(row=1, column=0, pady=(10, 0), sticky="ew")
        self.serial_disconnect_button = ttk.Button(
            serial_frame,
            text="Disconnect",
            command=self.disconnect_serial,
            state="disabled",
        )
        self.serial_disconnect_button.grid(
            row=1, column=1, pady=(10, 0), padx=(8, 12), sticky="w"
        )
        ttk.Label(serial_frame, textvariable=self.serial_status).grid(
            row=1, column=2, columnspan=3, pady=(10, 0), sticky="w"
        )

        trim_frame = ttk.LabelFrame(container, text="G-code Trim", padding=10)
        trim_frame.grid(row=2, column=0, sticky="ew", pady=(12, 0))

        ttk.Label(trim_frame, text="X adjustment:").grid(row=0, column=0, sticky="w")
        ttk.Entry(trim_frame, textvariable=self.x_trim_step, width=9).grid(
            row=0, column=1, padx=(8, 6), sticky="w"
        )
        ttk.Button(
            trim_frame, text="X -", command=lambda: self.adjust_trim("x", -1)
        ).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(
            trim_frame, text="X +", command=lambda: self.adjust_trim("x", 1)
        ).grid(row=0, column=3)

        ttk.Label(trim_frame, text="Y adjustment:").grid(
            row=0, column=4, padx=(24, 0), sticky="w"
        )
        ttk.Entry(trim_frame, textvariable=self.y_trim_step, width=9).grid(
            row=0, column=5, padx=(8, 6), sticky="w"
        )
        ttk.Button(
            trim_frame, text="Y -", command=lambda: self.adjust_trim("y", -1)
        ).grid(row=0, column=6, padx=(0, 6))
        ttk.Button(
            trim_frame, text="Y +", command=lambda: self.adjust_trim("y", 1)
        ).grid(row=0, column=7)

        ttk.Label(trim_frame, text="Current trim:").grid(
            row=1, column=0, pady=(10, 0), sticky="w"
        )
        ttk.Label(trim_frame, textvariable=self.trim_status).grid(
            row=1, column=1, columnspan=5, pady=(10, 0), padx=(8, 0), sticky="w"
        )
        ttk.Button(trim_frame, text="Reset trim", command=self.reset_trim).grid(
            row=1, column=6, columnspan=2, pady=(10, 0), sticky="e"
        )

        manual_frame = ttk.LabelFrame(container, text="Manual Serial Command", padding=10)
        manual_frame.grid(row=3, column=0, sticky="ew", pady=(12, 0))
        manual_frame.columnconfigure(0, weight=1)
        self.manual_command_entry = ttk.Entry(
            manual_frame, textvariable=self.manual_command, state="disabled"
        )
        self.manual_command_entry.grid(row=0, column=0, sticky="ew")
        self.manual_command_entry.bind("<Return>", self.send_manual_command)
        self.manual_send_button = ttk.Button(
            manual_frame,
            text="Send",
            command=self.send_manual_command,
            state="disabled",
        )
        self.manual_send_button.grid(row=0, column=1, padx=(8, 0))
        ttk.Label(manual_frame, textvariable=self.manual_status).grid(
            row=1, column=0, columnspan=2, pady=(8, 0), sticky="w"
        )

        output_frame = ttk.LabelFrame(container, text="Output", padding=10)
        output_frame.grid(row=4, column=0, sticky="nsew", pady=(12, 0))
        output_frame.columnconfigure(1, weight=1)
        output_frame.rowconfigure(1, weight=1)
        container.rowconfigure(4, weight=1)

        ttk.Label(output_frame, text="Latest G-code:").grid(row=0, column=0, sticky="w")
        ttk.Label(output_frame, textvariable=self.last_gcode).grid(
            row=0, column=1, sticky="w", padx=(8, 0)
        )
        self.output_log = scrolledtext.ScrolledText(
            output_frame, height=12, state="disabled", wrap="word"
        )
        self.output_log.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(8, 0))

    def _log(self, message: str) -> None:
        self.output_log.configure(state="normal")
        self.output_log.insert("end", message + "\n")
        self.output_log.see("end")
        self.output_log.configure(state="disabled")

    def _queue_event(self, event_type: str, message: str) -> None:
        self.events.put((event_type, message))

    def adjust_trim(self, axis: str, direction: int) -> None:
        step_variable = self.x_trim_step if axis == "x" else self.y_trim_step
        try:
            step = float(step_variable.get())
            if not math.isfinite(step) or step <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "Trim error", f"{axis.upper()} adjustment must be a positive number."
            )
            return

        with self.trim_lock:
            if axis == "x":
                self.x_trim += direction * step
            else:
                self.y_trim += direction * step
            x_trim, y_trim = self.x_trim, self.y_trim
        self._update_trim_status(x_trim, y_trim)

    def reset_trim(self) -> None:
        with self.trim_lock:
            self.x_trim = 0.0
            self.y_trim = 0.0
        self._update_trim_status(0.0, 0.0)

    def _update_trim_status(self, x_trim: float, y_trim: float) -> None:
        status = (
            f"X {format_signed_number(x_trim)} degrees, "
            f"Y {format_signed_number(y_trim)} degrees"
        )
        self.trim_status.set(status)
        self._log(f"Trim changed: {status}")

    def _process_events(self) -> None:
        try:
            while True:
                event_type, message = self.events.get_nowait()
                if event_type == "log":
                    self._log(message)
                elif event_type == "gcode":
                    self.last_gcode.set(message)
                    self._log(message)
                elif event_type == "satdump_status":
                    self.satdump_status.set(message)
                elif event_type == "serial_error":
                    self._handle_serial_error(message)
                elif event_type == "tcp_client":
                    self.tcp_client_active = message == "connected"
                    self._update_manual_controls()
        except queue.Empty:
            pass
        if self.root.winfo_exists():
            self.root.after(self.POLL_INTERVAL_MS, self._process_events)

    def refresh_serial_ports(self, show_error: bool = True) -> None:
        ports = available_serial_ports()
        self.serial_port_combo["values"] = ports
        if ports and not self.serial_port_name.get():
            self.serial_port_name.set(ports[0])
        if not ports and show_error:
            messagebox.showinfo(
                "No serial ports",
                "No serial ports were found. You can type a port name manually.\n\n"
                "If pyserial is not installed, run:\npython -m pip install pyserial",
            )

    def connect_serial(self) -> None:
        port_name = self.serial_port_name.get().strip()
        if not port_name:
            messagebox.showerror("Serial error", "Select or enter a serial port.")
            return
        try:
            baud_rate = int(self.baud_rate.get())
            if baud_rate <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Serial error", "Baud rate must be a positive integer.")
            return

        try:
            serial_port = open_serial(port_name, baud_rate)
        except RuntimeError as error:
            messagebox.showerror("Serial error", str(error))
            return

        with self.serial_lock:
            self.serial_port = serial_port
        self.serial_status.set(f"Connected to {port_name} at {baud_rate} baud")
        self.serial_connect_button.configure(state="disabled")
        self.serial_disconnect_button.configure(state="normal")
        self.serial_port_combo.configure(state="disabled")
        self.baud_combo.configure(state="disabled")
        self.refresh_button.configure(state="disabled")
        self._log(self.serial_status.get())
        self._update_manual_controls()

    def disconnect_serial(self) -> None:
        with self.serial_lock:
            serial_port, self.serial_port = self.serial_port, None
            if serial_port is not None:
                try:
                    serial_port.close()
                except OSError:
                    pass
        self.serial_status.set("Disconnected")
        self.serial_connect_button.configure(state="normal")
        self.serial_disconnect_button.configure(state="disabled")
        self.serial_port_combo.configure(state="normal")
        self.baud_combo.configure(state="normal")
        self.refresh_button.configure(state="normal")
        self._log("Serial disconnected")
        self._update_manual_controls()

    def _handle_serial_error(self, message: str) -> None:
        self.disconnect_serial()
        self._log(f"Serial error: {message}")

    def _update_manual_controls(self) -> None:
        serial_connected = self.serial_port is not None
        tcp_connected = self.tcp_client_active or self.client_socket is not None
        if tcp_connected:
            enabled = False
            status = "Unavailable while SatDump is connected"
        elif not serial_connected:
            enabled = False
            status = "Connect serial port to send"
        else:
            enabled = True
            status = "Ready"

        state = "normal" if enabled else "disabled"
        self.manual_command_entry.configure(state=state)
        self.manual_send_button.configure(state=state)
        self.manual_status.set(status)

    def send_manual_command(self, _event=None) -> None:
        if self.tcp_client_active or self.client_socket is not None:
            messagebox.showerror(
                "Manual command unavailable",
                "Disconnect the SatDump TCP client before sending manual commands.",
            )
            return

        command = self.manual_command.get().strip()
        if not command:
            return
        if not self._write_serial(command):
            if self.serial_port is None:
                messagebox.showerror("Serial error", "Connect the serial port first.")
            return

        print(command, flush=True)
        self._log(f"Manual: {command}")
        self.manual_command.set("")

    def connect_satdump(self) -> None:
        address = self.bind_address.get().strip()
        try:
            port = int(self.tcp_port.get())
            if not 1 <= port <= 65535:
                raise ValueError
        except ValueError:
            messagebox.showerror("SatDump error", "TCP port must be between 1 and 65535.")
            return

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(0.5)
        try:
            server.bind((address, port))
            server.listen()
        except OSError as error:
            server.close()
            messagebox.showerror("SatDump error", f"Could not listen on {address}:{port}: {error}")
            return

        self.server_socket = server
        self.listener_description = f"{address}:{port}"
        self.server_stop.clear()
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()
        self.satdump_status.set(f"Listening on {self.listener_description}")
        self.satdump_connect_button.configure(state="disabled")
        self.satdump_disconnect_button.configure(state="normal")
        self.bind_entry.configure(state="disabled")
        self.tcp_port_entry.configure(state="disabled")
        self._log(self.satdump_status.get())

    def disconnect_satdump(self) -> None:
        self.server_stop.set()
        for active_socket in (self.client_socket, self.server_socket):
            if active_socket is not None:
                try:
                    active_socket.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    active_socket.close()
                except OSError:
                    pass
        self.client_socket = None
        self.server_socket = None
        if (
            self.server_thread is not None
            and self.server_thread is not threading.current_thread()
        ):
            self.server_thread.join(timeout=1)
        self.server_thread = None
        self.tcp_client_active = False
        self._update_manual_controls()
        self.satdump_status.set("Disconnected")
        self.satdump_connect_button.configure(state="normal")
        self.satdump_disconnect_button.configure(state="disabled")
        self.bind_entry.configure(state="normal")
        self.tcp_port_entry.configure(state="normal")
        self._log("SatDump TCP disconnected")

    def _server_loop(self) -> None:
        server = self.server_socket
        if server is None:
            return
        while not self.server_stop.is_set():
            try:
                client, address = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.client_socket = client
            client.settimeout(0.5)
            peer = f"{address[0]}:{address[1]}"
            self._queue_event("satdump_status", f"Connected to SatDump at {peer}")
            self._queue_event("tcp_client", "connected")
            self._queue_event("log", f"SatDump connected from {peer}")
            try:
                self._handle_client(client)
            except (ConnectionError, OSError) as error:
                if not self.server_stop.is_set():
                    self._queue_event("log", f"SatDump connection error: {error}")
            finally:
                try:
                    client.close()
                except OSError:
                    pass
                if self.client_socket is client:
                    self.client_socket = None

            if not self.server_stop.is_set():
                self._queue_event("tcp_client", "disconnected")
                self._queue_event(
                    "satdump_status", f"Listening on {self.listener_description}"
                )
                self._queue_event("log", "SatDump client disconnected")

    def _handle_client(self, client: socket.socket) -> None:
        pending = b""
        while not self.server_stop.is_set():
            try:
                data = client.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            pending += data

            while b"\n" in pending:
                raw_command, pending = pending.split(b"\n", 1)
                try:
                    command = raw_command.decode("ascii").strip().strip("\r")
                except UnicodeDecodeError as error:
                    self._queue_event("log", f"Ignoring non-ASCII command: {error}")
                    client.sendall(b"RPRT -1\n")
                    continue
                if not command:
                    continue
                self._handle_command(client, command)

    def _handle_command(self, client: socket.socket, command: str) -> None:
        if command == "p":
            azimuth, elevation = self.last_position
            response = f"{format_number(azimuth)}\n{format_number(elevation)}\n"
            client.sendall(response.encode("ascii"))
            return

        try:
            position = parse_set_position(command)
        except (OverflowError, ValueError) as error:
            self._queue_event("log", f"Ignoring TCP command {command!r}: {error}")
            client.sendall(b"RPRT -1\n")
            return

        with self.trim_lock:
            x_trim, y_trim = self.x_trim, self.y_trim
        gcode = position_to_gcode(*position, x_trim=x_trim, y_trim=y_trim)
        print(gcode, flush=True)
        self._queue_event("gcode", gcode)

        if self._write_serial(gcode):
            self.last_position = position
            client.sendall(b"RPRT 0\n")
        else:
            self._queue_event("log", "G-code not sent: serial port is disconnected")
            client.sendall(b"RPRT -1\n")

    def _write_serial(self, gcode: str) -> bool:
        with self.serial_lock:
            if self.serial_port is None:
                return False
            try:
                self.serial_port.write((gcode + "\n").encode("ascii"))
                self.serial_port.flush()
            except Exception as error:
                self._queue_event("serial_error", str(error))
                return False
        return True

    def close(self) -> None:
        self.disconnect_satdump()
        self.disconnect_serial()
        self.root.destroy()


def main() -> int:
    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f"error: could not start Tkinter: {error}", file=sys.stderr)
        return 1
    SatelliteTrackerGUI(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
