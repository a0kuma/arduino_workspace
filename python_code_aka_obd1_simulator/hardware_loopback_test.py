#!/usr/bin/env python3
"""End-to-end Raspberry Pi GPIO21 -> Arduino D7 Toyota OBD1 loopback test."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Sequence

import serial

from toyota_obd1_sim import (
    PROFILES,
    build_timeline,
    drive_gpio,
    format_bytes,
    write_gtkw,
    write_vcd,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_VCD = PROJECT_ROOT / "waveforms" / "hardware_loopback_gpio21.vcd"
DEFAULT_GTKW = PROJECT_ROOT / "waveforms" / "hardware_loopback_gpio21.gtkw"


def read_available_lines(port: serial.Serial, seconds: float) -> list[str]:
    lines: list[str] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = port.readline()
        if raw:
            line = raw.decode("ascii", errors="replace").strip()
            if line:
                lines.append(line)
                print(line)
    return lines


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the physical GPIO21-to-D7 loopback test.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--gpio", type=int, default=21)
    parser.add_argument("--profile", choices=sorted(PROFILES), default="cruise")
    parser.add_argument("--packet-id", type=lambda value: int(value, 0), default=0x0)
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--bit-ms", type=float, default=8.0)
    parser.add_argument("--preamble-ms", type=float, default=160.0)
    parser.add_argument("--sync-low-ms", type=float, default=1.0)
    parser.add_argument("--reset-wait", type=float, default=2.5)
    parser.add_argument("--read-after", type=float, default=2.0)
    parser.add_argument("--vcd", type=Path, default=DEFAULT_VCD)
    parser.add_argument("--gtkw", type=Path, default=DEFAULT_GTKW)
    parser.add_argument(
        "--cleanup-gpio",
        action="store_true",
        help="release GPIO after test; default leaves the final LOW level stable",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    data = PROFILES[args.profile]
    expected_raw = format_bytes(data)
    packets = [(args.packet_id, data) for _ in range(args.count)]
    timeline = build_timeline(
        packets,
        bit_us=int(round(args.bit_ms * 1000)),
        preamble_us=int(round(args.preamble_ms * 1000)),
        sync_low_us=int(round(args.sync_low_ms * 1000)),
    )

    write_vcd(args.vcd, timeline)
    write_gtkw(args.gtkw, args.vcd)

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        time.sleep(args.reset_wait)
        print("startup:")
        read_available_lines(port, 0.5)
        port.reset_input_buffer()

        backend = drive_gpio(args.gpio, timeline, cleanup=args.cleanup_gpio)
        print(f"gpio_backend={backend}")
        print("receiver:")
        lines = read_available_lines(port, args.read_after)

    matched = [line for line in lines if line.startswith("PACKET ") and expected_raw in line]
    if not matched:
        print(f"expected raw packet not seen: {expected_raw}", file=sys.stderr)
        return 1

    print(f"PASS packets_seen={len(matched)} expected_raw={expected_raw}")
    print(f"vcd={args.vcd}")
    print(f"gtkw={args.gtkw}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
