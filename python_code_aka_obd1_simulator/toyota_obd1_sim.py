#!/usr/bin/env python3
"""Toyota OBD1 VF1 signal simulator for Raspberry Pi GPIO.

Default wiring for this workspace:
  Raspberry Pi BCM GPIO21 -> Arduino D2
  Raspberry Pi GND        -> Arduino GND

The generated waveform matches the edge-duration decoder used by
hyperion11/toyota-obd-1: a long HIGH preamble, a falling edge to start a
packet, 8 ms bit cells, 4 ID bits, then 11-bit framed bytes
(LOW start, 8 data bits LSB-first, two HIGH stop bits).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


Bit = int
Transition = Tuple[int, Bit]  # time_us, level

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
DEFAULT_VCD = PROJECT_ROOT / "waveforms" / "toyota_obd1_gpio21.vcd"
DEFAULT_GTKW = PROJECT_ROOT / "waveforms" / "toyota_obd1_gpio21.gtkw"

TOYOTA_MAX_BYTES = 24

PROFILES = {
    # Raw bytes chosen to decode close to the sample Data10.csv cruise rows:
    # INJ 2.25 ms, IGN 18.4 deg, IAC 37.3 %, RPM 1100, ECT 80 C, SPD 60 km/h.
    "cruise": [0x00, 0x12, 0x67, 0x5F, 0x2C, 0x01, 0xE4, 0x00,
               0x3C, 0x80, 0x00, 0x20, 0x82, 0x08, 0x30],
    "idle": [0x00, 0x18, 0x55, 0x50, 0x20, 0x14, 0xD8, 0x12,
             0x00, 0x40, 0x00, 0x22, 0x82, 0x00, 0x10],
}


def parse_data_bytes(text: str) -> List[int]:
    parts = text.replace(",", " ").split()
    if not parts:
        raise ValueError("data byte list is empty")
    values: List[int] = []
    for part in parts:
        value = int(part, 0)
        if not 0 <= value <= 0xFF:
            raise ValueError(f"byte out of range: {part}")
        values.append(value)
    if len(values) > TOYOTA_MAX_BYTES:
        raise ValueError(f"maximum packet size is {TOYOTA_MAX_BYTES} bytes")
    return values


def packet_bits(packet_id: int, data: Sequence[int]) -> List[Bit]:
    if not 0 <= packet_id <= 0x0F:
        raise ValueError("packet ID must fit in 4 bits")
    if len(data) > TOYOTA_MAX_BYTES:
        raise ValueError(f"maximum packet size is {TOYOTA_MAX_BYTES} bytes")

    bits: List[Bit] = [(packet_id >> bit) & 1 for bit in range(4)]
    for byte in data:
        if not 0 <= byte <= 0xFF:
            raise ValueError(f"byte out of range: {byte}")
        bits.append(0)  # start bit
        bits.extend((byte >> bit) & 1 for bit in range(8))
        bits.extend([1, 1])
    return bits


def runs(bits: Sequence[Bit]) -> Iterable[Tuple[Bit, int]]:
    if not bits:
        return
    level = bits[0]
    count = 1
    for bit in bits[1:]:
        if bit == level:
            count += 1
        else:
            yield level, count
            level = bit
            count = 1
    yield level, count


def add_transition(transitions: List[Transition], time_us: int, level: Bit) -> None:
    if transitions and transitions[-1][1] == level:
        return
    transitions.append((time_us, level))


def build_timeline(
    packets: Sequence[Tuple[int, Sequence[int]]],
    bit_us: int,
    preamble_us: int,
    sync_low_us: int,
) -> List[Transition]:
    """Build GPIO transitions.

    A final falling edge is emitted after the last packet's HIGH trailer so the
    Arduino decoder can commit the last packet, exactly like the next packet's
    falling preamble edge would do on a live Toyota data stream.
    """
    if bit_us <= 0:
        raise ValueError("bit_us must be positive")
    if preamble_us <= 15 * bit_us:
        raise ValueError("preamble must be longer than 15 bit cells")
    if not packets:
        raise ValueError("at least one packet is required")

    t = 0
    current = 1
    transitions: List[Transition] = [(0, current)]
    t += preamble_us

    for packet_id, data in packets:
        bits = packet_bits(packet_id, data)

        current = 0
        add_transition(transitions, t, current)

        if bits[0] == 1:
            t += sync_low_us
            current = 1
            add_transition(transitions, t, current)

        for level, count in runs(bits):
            if current != level:
                current = level
                add_transition(transitions, t, current)
            t += count * bit_us

        if current != 1:
            current = 1
            add_transition(transitions, t, current)
        t += preamble_us

    add_transition(transitions, t, 0)
    return transitions


@dataclass
class DecodedPacket:
    packet_id: int
    data: List[int]


class ArduinoTimingDecoder:
    """Software mirror of the Arduino ChangeState() timing state machine."""

    def __init__(self) -> None:
        self.in_packet = False
        self.start_ms = 0
        self.bit_count = 0
        self.packet_id = 0
        self.edata = [0] * TOYOTA_MAX_BYTES
        self.fail_bits: List[int] = []
        self.packets: List[DecodedPacket] = []

    def edge(self, time_us: int, state: Bit) -> None:
        now_ms = time_us // 1000
        if not self.in_packet:
            if state == 1:
                self.start_ms = now_ms
            elif now_ms - self.start_ms > 15 * 8:
                self.start_ms = now_ms
                self.in_packet = True
                self.bit_count = 0
            return

        bits = ((now_ms - self.start_ms) + 1) // 8
        self.start_ms = now_ms

        while bits > 0:
            if self.bit_count < 4:
                if self.bit_count == 0:
                    self.packet_id = 0
                self.packet_id >>= 1
                if state == 0:
                    self.packet_id |= 0x08
            else:
                bitpos = (self.bit_count - 4) % 11
                bytepos = (self.bit_count - 4) // 11

                if bitpos == 0:
                    if self.bit_count > 4 and state != 1:
                        self.fail_bits.append(self.bit_count)
                        self.in_packet = False
                        break
                elif bitpos < 9:
                    self.edata[bytepos] >>= 1
                    if state == 0:
                        self.edata[bytepos] |= 0x80
                else:
                    if state != 0:
                        self.fail_bits.append(self.bit_count)
                        self.in_packet = False
                        break
                    if bitpos == 10 and (bits > 1 or bytepos == TOYOTA_MAX_BYTES - 1):
                        self.packets.append(
                            DecodedPacket(self.packet_id, self.edata[: bytepos + 1])
                        )
                        if bits >= 16:
                            self.bit_count = 0
                        else:
                            self.fail_bits.append(self.bit_count)
                            self.in_packet = False
                        break

            self.bit_count += 1
            bits -= 1


def decode_timeline(transitions: Sequence[Transition]) -> Tuple[List[DecodedPacket], List[int]]:
    decoder = ArduinoTimingDecoder()
    for time_us, level in transitions[1:]:
        decoder.edge(time_us, level)
    return decoder.packets, decoder.fail_bits


def write_vcd(path: Path, transitions: Sequence[Transition]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as fh:
        fh.write("$date\n")
        fh.write(f"  {time.strftime('%Y-%m-%d %H:%M:%S %z')}\n")
        fh.write("$end\n")
        fh.write("$version\n")
        fh.write("  toyota_obd1_sim.py\n")
        fh.write("$end\n")
        fh.write("$timescale 1 us $end\n")
        fh.write("$scope module toyota_obd1 $end\n")
        fh.write("$var wire 1 ! gpio21_to_d2 $end\n")
        fh.write("$upscope $end\n")
        fh.write("$enddefinitions $end\n")
        fh.write("$dumpvars\n")
        fh.write(f"{transitions[0][1]}!\n")
        fh.write("$end\n")
        for time_us, level in transitions[1:]:
            fh.write(f"#{time_us}\n")
            fh.write(f"{level}!\n")


def write_gtkw(path: Path, vcd_path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as fh:
        fh.write("[*]\n")
        fh.write(f"[dumpfile] {vcd_path}\n")
        fh.write("[dumpfile_mtime] 0\n")
        fh.write("[timestart] 0\n")
        fh.write("[size] 1200 700\n")
        fh.write("[pos] -1 -1\n")
        fh.write("@28\n")
        fh.write("toyota_obd1.gpio21_to_d2\n")


def sleep_until(deadline: float) -> None:
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        if remaining > 0.002:
            time.sleep(remaining - 0.001)


class RpiGpioDriver:
    def __init__(self, pin: int) -> None:
        self.pin = pin
        self.backend = "RPi.GPIO"
        self.gpio = None
        self.lgpio = None
        self.chip = None

    def __enter__(self) -> "RpiGpioDriver":
        try:
            import RPi.GPIO as gpio  # type: ignore

            self.gpio = gpio
            gpio.setmode(gpio.BCM)
            gpio.setwarnings(False)
            gpio.setup(self.pin, gpio.OUT, initial=gpio.HIGH)
        except Exception:
            import lgpio  # type: ignore

            self.backend = "lgpio"
            self.lgpio = lgpio
            self.chip = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(self.chip, self.pin, 1)
        return self

    def write(self, level: Bit) -> None:
        if self.gpio is not None:
            self.gpio.output(self.pin, self.gpio.HIGH if level else self.gpio.LOW)
        else:
            self.lgpio.gpio_write(self.chip, self.pin, int(level))

    def close(self, cleanup: bool) -> None:
        if self.gpio is not None:
            if cleanup:
                self.gpio.cleanup(self.pin)
        elif self.lgpio is not None and self.chip is not None:
            if cleanup:
                self.lgpio.gpiochip_close(self.chip)

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close(cleanup=True)


def drive_gpio(pin: int, transitions: Sequence[Transition], cleanup: bool) -> str:
    driver = RpiGpioDriver(pin)
    driver.__enter__()
    try:
        backend = driver.backend
        start = time.monotonic()
        for time_us, level in transitions:
            sleep_until(start + time_us / 1_000_000.0)
            driver.write(level)
        return backend
    finally:
        driver.close(cleanup=cleanup)


def format_bytes(data: Sequence[int]) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def make_packets(args: argparse.Namespace) -> List[Tuple[int, List[int]]]:
    data = parse_data_bytes(args.data) if args.data else list(PROFILES[args.profile])
    count = args.count
    if args.duration:
        frame_bits = len(packet_bits(args.packet_id, data))
        frame_us = args.preamble_ms * 1000 + frame_bits * args.bit_ms * 1000
        frame_us += args.preamble_ms * 1000
        count = max(count, math.ceil(args.duration * 1_000_000 / frame_us) + 1)
    return [(args.packet_id, data) for _ in range(count)]


def run_self_test() -> int:
    packets_in = [(0x0, PROFILES["cruise"]), (0x0, PROFILES["idle"])]
    transitions = build_timeline(packets_in, bit_us=8000, preamble_us=160000, sync_low_us=1000)
    packets_out, fail_bits = decode_timeline(transitions)
    expected = [DecodedPacket(pid, list(data)) for pid, data in packets_in]
    if packets_out != expected or fail_bits:
        print("SELF_TEST failed", file=sys.stderr)
        print(f"decoded={packets_out}", file=sys.stderr)
        print(f"fail_bits={fail_bits}", file=sys.stderr)
        return 1
    print("SELF_TEST ok")
    print(f"decoded_packets={len(packets_out)}")
    print(f"first_packet_id=0x{packets_out[0].packet_id:X}")
    print(f"first_packet_raw={format_bytes(packets_out[0].data)}")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Drive a Toyota OBD1-style VF1 waveform from Raspberry Pi GPIO21."
    )
    parser.add_argument("--gpio", type=int, default=21, help="BCM GPIO output pin")
    parser.add_argument("--packet-id", type=lambda value: int(value, 0), default=0x0)
    parser.add_argument("--profile", choices=sorted(PROFILES), default="cruise")
    parser.add_argument("--data", help="raw data bytes, e.g. '0x00 0x12 0x67'")
    parser.add_argument("--count", type=int, default=4, help="number of packets to emit")
    parser.add_argument("--duration", type=float, help="minimum transmit duration in seconds")
    parser.add_argument("--bit-ms", type=float, default=8.0, help="bit cell length")
    parser.add_argument("--preamble-ms", type=float, default=160.0, help="HIGH preamble/trailer")
    parser.add_argument(
        "--sync-low-ms",
        type=float,
        default=1.0,
        help="short LOW sync gap when the first ID bit is HIGH",
    )
    parser.add_argument("--vcd", type=Path, default=DEFAULT_VCD)
    parser.add_argument("--gtkw", type=Path, default=DEFAULT_GTKW)
    parser.add_argument("--dry-run", action="store_true", help="write VCD and decode, but do not use GPIO")
    parser.add_argument("--self-test", action="store_true", help="run built-in protocol test")
    parser.add_argument(
        "--no-cleanup",
        action="store_true",
        help="leave GPIO configured after transmit; useful to avoid an end-of-run float",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if args.self_test:
        return run_self_test()

    if args.count <= 0:
        parser.error("--count must be positive")
    bit_us = int(round(args.bit_ms * 1000))
    preamble_us = int(round(args.preamble_ms * 1000))
    sync_low_us = int(round(args.sync_low_ms * 1000))

    packets = make_packets(args)
    transitions = build_timeline(packets, bit_us, preamble_us, sync_low_us)
    decoded, fail_bits = decode_timeline(transitions)

    write_vcd(args.vcd, transitions)
    write_gtkw(args.gtkw, args.vcd)

    print(f"packet_id=0x{packets[0][0]:X}")
    print(f"data={format_bytes(packets[0][1])}")
    print(f"packets_requested={len(packets)} decoded_by_reference={len(decoded)}")
    print(f"vcd={args.vcd}")
    print(f"gtkw={args.gtkw}")

    if fail_bits:
        print(f"reference_decoder_fail_bits={fail_bits}", file=sys.stderr)
        return 1

    if args.dry_run:
        return 0

    backend = drive_gpio(args.gpio, transitions, cleanup=not args.no_cleanup)
    print(f"gpio_backend={backend} gpio_bcm={args.gpio}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
