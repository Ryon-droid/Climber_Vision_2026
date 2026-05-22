#!/usr/bin/env python3
"""Mock lobshot serial sink for RM 0x0310 outer frames or raw inner packets."""

from __future__ import annotations

import argparse
import os
import pty
import select
import struct
import sys
import termios
import time
import tty
from typing import Optional


OUTER_FRAME_SIZE = 309
INNER_PACKET_SIZE = 300
OUTER_CMD_ID = 0x0310
CRC8_INIT = 0xFF
CRC16_INIT = 0xFFFF

CRC8_TABLE = (
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
)

CRC16_TABLE = (
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A, 0xBED3,
    0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102, 0x308B, 0x0210, 0x1399,
    0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50,
    0xFBEF, 0xEA66, 0xD8FD, 0xC974, 0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3, 0x5285, 0x430C, 0x7197, 0x601E,
    0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5,
    0xA96A, 0xB8E3, 0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70, 0x8408, 0x9581, 0xA71A, 0xB693,
    0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948, 0x3BD3, 0x2A5A,
    0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD, 0xB58B, 0xA402, 0x9699, 0x8710,
    0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF,
    0x0C60, 0x1DE9, 0x2F72, 0x3EFB, 0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A, 0xE70E, 0xF687, 0xC41C, 0xD595,
    0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C,
    0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
)


def set_raw(fd: int) -> None:
    tty.setraw(fd, when=termios.TCSANOW)


def crc8(data: bytes) -> int:
    crc = CRC8_INIT
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


def crc16(data: bytes) -> int:
    crc = CRC16_INIT
    for byte in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


def ascii_preview(data: bytes) -> str:
    chars = []
    for byte in data:
        if 32 <= byte <= 126:
            chars.append(chr(byte))
        elif byte == 0:
            continue
        else:
            chars.append(".")
    return "".join(chars).strip()


def decode_inner_packet(inner: bytes) -> tuple[int, int, int, str]:
    frame_no = int.from_bytes(inner[0:2], "little")
    frag_no = int.from_bytes(inner[2:4], "little")
    total_bytes = int.from_bytes(inner[4:8], "little")
    preview = ascii_preview(inner[8:40])
    return frame_no, frag_no, total_bytes, preview


class MockLobshotSerial:
    def __init__(self, inner_only: bool, port_file: Optional[str]) -> None:
        self.inner_only = inner_only
        self.port_file = port_file
        self.master_fd: Optional[int] = None
        self.slave_fd: Optional[int] = None
        self.slave_name: Optional[str] = None
        self.packet_count = 0
        self.last_stats_at = time.monotonic()
        self.last_seq: Optional[int] = None

    def open(self) -> None:
        self.master_fd, self.slave_fd = pty.openpty()
        set_raw(self.master_fd)
        set_raw(self.slave_fd)
        self.slave_name = os.ttyname(self.slave_fd)
        if self.port_file:
            with open(self.port_file, "w", encoding="utf-8") as file:
                file.write(self.slave_name + "\n")

    def close(self) -> None:
        for fd in (self.master_fd, self.slave_fd):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass

    def handle_inner_packet(self, packet: bytes) -> None:
        self.packet_count += 1
        frame_no, frag_no, total_bytes, preview = decode_inner_packet(packet)
        if self.packet_count <= 5 or self.packet_count % 50 == 0:
            print(
                "[mock_lobshot] INNER "
                f"packet={self.packet_count} frame={frame_no} frag={frag_no} total={total_bytes} "
                f"preview='{preview}'",
                flush=True,
            )

    def handle_outer_frame(self, frame: bytes) -> None:
        self.packet_count += 1

        if frame[0] != 0xA5:
            print("[mock_lobshot] Drop frame: invalid SOF", flush=True)
            return

        data_length = int.from_bytes(frame[1:3], "little")
        seq = frame[3]
        header_crc8 = frame[4]
        cmd_id = int.from_bytes(frame[5:7], "little")
        payload = frame[7:307]
        frame_crc16 = int.from_bytes(frame[307:309], "little")

        crc8_ok = crc8(frame[:4]) == header_crc8
        crc16_ok = crc16(frame[:307]) == frame_crc16

        frame_no, frag_no, total_bytes, preview = decode_inner_packet(payload)
        seq_gap = "n/a" if self.last_seq is None else str((seq - self.last_seq) & 0xFF)
        self.last_seq = seq

        if self.packet_count <= 5 or self.packet_count % 50 == 0 or not (crc8_ok and crc16_ok):
            print(
                "[mock_lobshot] OUTER "
                f"packet={self.packet_count} seq={seq} seq_gap={seq_gap} len={data_length} "
                f"cmd=0x{cmd_id:04X} crc8={'ok' if crc8_ok else 'bad'} crc16={'ok' if crc16_ok else 'bad'} "
                f"frame={frame_no} frag={frag_no} total={total_bytes} preview='{preview}'",
                flush=True,
            )

        if data_length != INNER_PACKET_SIZE:
            print(f"[mock_lobshot] Warning: unexpected data_length={data_length}", flush=True)
        if cmd_id != OUTER_CMD_ID:
            print(f"[mock_lobshot] Warning: unexpected cmd_id=0x{cmd_id:04X}", flush=True)

    def run(self) -> int:
        self.open()
        assert self.master_fd is not None
        assert self.slave_name is not None

        frame_mode = "inner_only" if self.inner_only else "rm_outer"
        print(
            f"[mock_lobshot] PTY ready: {self.slave_name}\n"
            f"[mock_lobshot] Use this as lobshot_com_port in configs/lobshot.yaml.\n"
            f"[mock_lobshot] Expected mode: {frame_mode}",
            flush=True,
        )
        if self.port_file:
            print(f"[mock_lobshot] Port path saved to {self.port_file}", flush=True)

        buffer = bytearray()
        packet_size = INNER_PACKET_SIZE if self.inner_only else OUTER_FRAME_SIZE

        try:
            while True:
                ready, _, _ = select.select([self.master_fd], [], [], 0.2)
                if not ready:
                    self.maybe_print_idle_stats()
                    continue

                chunk = os.read(self.master_fd, 4096)
                if not chunk:
                    self.maybe_print_idle_stats()
                    continue
                buffer.extend(chunk)

                while len(buffer) >= packet_size:
                    if self.inner_only:
                        packet = bytes(buffer[:packet_size])
                        del buffer[:packet_size]
                        self.handle_inner_packet(packet)
                        continue

                    sof_index = buffer.find(b"\xA5")
                    if sof_index < 0:
                        buffer.clear()
                        break
                    if sof_index > 0:
                        del buffer[:sof_index]
                    if len(buffer) < packet_size:
                        break

                    frame = bytes(buffer[:packet_size])
                    del buffer[:packet_size]
                    self.handle_outer_frame(frame)
        except KeyboardInterrupt:
            print("\n[mock_lobshot] Stopped by user", flush=True)
        finally:
            self.close()

        return 0

    def maybe_print_idle_stats(self) -> None:
        now = time.monotonic()
        if now - self.last_stats_at < 1.0:
            return
        self.last_stats_at = now
        print(f"[mock_lobshot] Waiting... packets={self.packet_count}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mock lobshot serial sink for RM 0x0310 frames.")
    parser.add_argument(
        "--inner-only",
        action="store_true",
        help="Expect 300-byte inner packets instead of 309-byte RM outer frames",
    )
    parser.add_argument("--port-file", type=str, default="", help="Optional file to write the PTY path into")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mock = MockLobshotSerial(inner_only=args.inner_only, port_file=args.port_file or None)
    return mock.run()


if __name__ == "__main__":
    sys.exit(main())
