#!/usr/bin/env python3
"""Mock gimbal serial endpoint for local vision testing.

Creates a PTY device that the C++ vision program can open as `com_port`.
The script periodically sends `io::GimbalToVision` packets and prints the
incoming `io::VisionToGimbal` packets from the vision side.
"""

from __future__ import annotations

import argparse
import math
import os
import pty
import select
import signal
import struct
import sys
import termios
import threading
import time
import tty
from typing import Optional


GV_PACKET_STRUCT = struct.Struct("<2sBffffffH2s")
VG_PACKET_STRUCT = struct.Struct("<2sBffffff2s")
GV_PACKET_SIZE = GV_PACKET_STRUCT.size
VG_PACKET_SIZE = VG_PACKET_STRUCT.size


def set_raw(fd: int) -> None:
    tty.setraw(fd, when=termios.TCSANOW)


class MockGimbalSerial:
    def __init__(
        self,
        mode: int,
        switch_after_s: float,
        switch_to_mode: Optional[int],
        cycle_modes: list[int],
        cycle_hold_s: float,
        bullet_speed: float,
        tx_rate_hz: float,
        yaw_amp_deg: float,
        yaw_period_s: float,
        pitch_deg: float,
        pitch_amp_deg: float,
        pitch_period_s: float,
        port_file: Optional[str],
    ) -> None:
        self.initial_mode = mode
        self.mode = mode
        self.switch_after_s = max(0.0, switch_after_s)
        self.switch_to_mode = switch_to_mode
        self.cycle_modes = cycle_modes
        self.cycle_hold_s = max(0.1, cycle_hold_s)
        self.bullet_speed = bullet_speed
        self.tx_rate_hz = max(1.0, tx_rate_hz)
        self.yaw_amp_rad = math.radians(yaw_amp_deg)
        self.yaw_period_s = max(0.1, yaw_period_s)
        self.pitch_center_rad = math.radians(pitch_deg)
        self.pitch_amp_rad = math.radians(pitch_amp_deg)
        self.pitch_period_s = max(0.1, pitch_period_s)
        self.port_file = port_file

        self.master_fd: Optional[int] = None
        self.slave_fd: Optional[int] = None
        self.slave_name: Optional[str] = None
        self.stop_event = threading.Event()
        self.bullet_count = 0
        self._last_rx_print = 0.0
        self._switched_mode = False
        self._cycle_index = 0
        self._next_cycle_switch_s = self.cycle_hold_s

    def open(self) -> None:
        self.master_fd, self.slave_fd = pty.openpty()
        set_raw(self.master_fd)
        set_raw(self.slave_fd)
        self.slave_name = os.ttyname(self.slave_fd)
        if self.port_file:
          with open(self.port_file, "w", encoding="utf-8") as f:
            f.write(self.slave_name + "\n")

    def close(self) -> None:
        for fd in (self.master_fd, self.slave_fd):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass

    def build_gv_packet(self, now: float) -> bytes:
        self.maybe_switch_mode(now)
        self.maybe_cycle_mode(now)
        yaw = self.yaw_amp_rad * math.sin(2.0 * math.pi * now / self.yaw_period_s)
        yaw_vel = (
            self.yaw_amp_rad
            * 2.0
            * math.pi
            / self.yaw_period_s
            * math.cos(2.0 * math.pi * now / self.yaw_period_s)
        )
        pitch = self.pitch_center_rad + self.pitch_amp_rad * math.sin(
            2.0 * math.pi * now / self.pitch_period_s
        )
        pitch_vel = (
            self.pitch_amp_rad
            * 2.0
            * math.pi
            / self.pitch_period_s
            * math.cos(2.0 * math.pi * now / self.pitch_period_s)
        )
        roll = 0.0
        return GV_PACKET_STRUCT.pack(
            b"GV",
            self.mode,
            yaw,
            yaw_vel,
            pitch,
            pitch_vel,
            roll,
            self.bullet_speed,
            self.bullet_count & 0xFFFF,
            b"EN",
        )

    def maybe_switch_mode(self, elapsed_s: float) -> None:
        if self.cycle_modes:
            return
        if self._switched_mode or self.switch_to_mode is None:
            return
        if elapsed_s < self.switch_after_s:
            return

        self.mode = self.switch_to_mode
        self._switched_mode = True
        print(
            "[mock_gimbal] Mode switch "
            f"{self.initial_mode} -> {self.mode} at t={elapsed_s:.2f}s",
            flush=True,
        )

    def maybe_cycle_mode(self, elapsed_s: float) -> None:
        if not self.cycle_modes:
            return
        if elapsed_s < self._next_cycle_switch_s:
            return

        self._cycle_index = (self._cycle_index + 1) % len(self.cycle_modes)
        previous_mode = self.mode
        self.mode = self.cycle_modes[self._cycle_index]
        print(
            "[mock_gimbal] Cycle switch "
            f"{previous_mode} -> {self.mode} at t={elapsed_s:.2f}s",
            flush=True,
        )
        self._next_cycle_switch_s += self.cycle_hold_s

    def send_loop(self) -> None:
        assert self.master_fd is not None
        interval = 1.0 / self.tx_rate_hz
        next_tick = time.monotonic()
        start = next_tick
        while not self.stop_event.is_set():
            now = time.monotonic()
            packet = self.build_gv_packet(now - start)
            try:
                os.write(self.master_fd, packet)
            except OSError:
                self.stop_event.set()
                break
            next_tick += interval
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.monotonic()

    def recv_loop(self) -> None:
        assert self.master_fd is not None
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                ready, _, _ = select.select([self.master_fd], [], [], 0.1)
            except (OSError, ValueError):
                self.stop_event.set()
                break
            if not ready:
                continue

            try:
                chunk = os.read(self.master_fd, 256)
            except OSError:
                self.stop_event.set()
                break
            if not chunk:
                continue
            buffer.extend(chunk)

            while len(buffer) >= VG_PACKET_SIZE:
                head_index = buffer.find(b"VG")
                if head_index < 0:
                    buffer.clear()
                    break
                if head_index > 0:
                    del buffer[:head_index]
                if len(buffer) < VG_PACKET_SIZE:
                    break

                packet = bytes(buffer[:VG_PACKET_SIZE])
                if packet[-2:] != b"EN":
                    del buffer[:2]
                    continue

                del buffer[:VG_PACKET_SIZE]
                self.handle_vg_packet(packet)

    def handle_vg_packet(self, packet: bytes) -> None:
        head, mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, tail = VG_PACKET_STRUCT.unpack(
            packet
        )
        if head != b"VG" or tail != b"EN":
            return

        fired = mode == 2
        if fired:
            self.bullet_count += 1

        now = time.monotonic()
        if now - self._last_rx_print < 0.02 and not fired:
            return
        self._last_rx_print = now

        print(
            "[mock_gimbal] RX "
            f"mode={mode} yaw={yaw:.4f} yaw_vel={yaw_vel:.4f} yaw_acc={yaw_acc:.4f} "
            f"pitch={pitch:.4f} pitch_vel={pitch_vel:.4f} pitch_acc={pitch_acc:.4f} "
            f"fire={'yes' if fired else 'no'} bullet_count={self.bullet_count}",
            flush=True,
        )

    def run(self) -> int:
        self.open()
        assert self.slave_name is not None
        print(
            f"[mock_gimbal] PTY ready: {self.slave_name}\n"
            f"[mock_gimbal] Use this as com_port in your yaml.",
            flush=True,
        )
        if self.cycle_modes:
            print(
                "[mock_gimbal] Cycling modes="
                f"{self.cycle_modes} every {self.cycle_hold_s:.2f}s",
                flush=True,
            )
        elif self.switch_to_mode is None:
            print(f"[mock_gimbal] Fixed mode={self.mode}", flush=True)
        else:
            print(
                "[mock_gimbal] Scheduled mode switch "
                f"{self.initial_mode} -> {self.switch_to_mode} after {self.switch_after_s:.2f}s",
                flush=True,
            )
        if self.port_file:
            print(f"[mock_gimbal] Port path saved to {self.port_file}", flush=True)

        tx_thread = threading.Thread(target=self.send_loop, daemon=True)
        rx_thread = threading.Thread(target=self.recv_loop, daemon=True)
        tx_thread.start()
        rx_thread.start()

        try:
            while not self.stop_event.is_set():
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass

        self.stop_event.set()
        tx_thread.join(timeout=1.0)
        rx_thread.join(timeout=1.0)
        self.close()
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mock gimbal serial endpoint for hero/infantry/sentry.")
    parser.add_argument("--mode", type=int, default=1, choices=[0, 1, 2], help="0 idle, 1 auto aim, 2 lobshot")
    parser.add_argument(
        "--switch-after-s",
        type=float,
        default=-1.0,
        help="Switch outgoing GV mode after this many seconds; negative disables switching",
    )
    parser.add_argument(
        "--switch-to-mode",
        type=int,
        default=2,
        choices=[0, 1, 2],
        help="Target mode used together with --switch-after-s",
    )
    parser.add_argument(
        "--cycle-modes",
        type=str,
        default="",
        help="Comma-separated repeating mode sequence, e.g. 1,2,1,2",
    )
    parser.add_argument(
        "--cycle-hold-s",
        type=float,
        default=5.0,
        help="Seconds to hold each mode when using --cycle-modes",
    )
    parser.add_argument("--bullet-speed", type=float, default=11.0, help="Reported bullet speed in m/s")
    parser.add_argument("--tx-rate-hz", type=float, default=100.0, help="Outgoing GV packet rate")
    parser.add_argument("--yaw-amp-deg", type=float, default=0.0, help="Yaw sine amplitude in degrees")
    parser.add_argument("--yaw-period-s", type=float, default=4.0, help="Yaw sine period")
    parser.add_argument("--pitch-deg", type=float, default=0.0, help="Pitch center in degrees")
    parser.add_argument("--pitch-amp-deg", type=float, default=0.0, help="Pitch sine amplitude in degrees")
    parser.add_argument("--pitch-period-s", type=float, default=5.0, help="Pitch sine period")
    parser.add_argument("--port-file", type=str, default="", help="Optional file to write the PTY path into")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cycle_modes: list[int] = []
    if args.cycle_modes.strip():
        cycle_modes = [int(part.strip()) for part in args.cycle_modes.split(",") if part.strip()]
        invalid_modes = [mode for mode in cycle_modes if mode not in (0, 1, 2)]
        if invalid_modes:
            raise ValueError(f"Invalid cycle mode(s): {invalid_modes}")
    switch_to_mode = args.switch_to_mode if args.switch_after_s >= 0.0 and not cycle_modes else None
    mock = MockGimbalSerial(
        mode=args.mode,
        switch_after_s=args.switch_after_s,
        switch_to_mode=switch_to_mode,
        cycle_modes=cycle_modes,
        cycle_hold_s=args.cycle_hold_s,
        bullet_speed=args.bullet_speed,
        tx_rate_hz=args.tx_rate_hz,
        yaw_amp_deg=args.yaw_amp_deg,
        yaw_period_s=args.yaw_period_s,
        pitch_deg=args.pitch_deg,
        pitch_amp_deg=args.pitch_amp_deg,
        pitch_period_s=args.pitch_period_s,
        port_file=args.port_file or None,
    )

    def handle_signal(signum: int, _frame: object) -> None:
        del signum
        mock.stop_event.set()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)
    return mock.run()


if __name__ == "__main__":
    sys.exit(main())
