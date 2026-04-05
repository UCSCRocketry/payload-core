#!/usr/bin/env python3
"""
Parse DATA.BIN from the payload recording system and convert to CSV.
"""

import struct
import csv
import sys
from pathlib import Path

SAMPLE_FMT = "<Q14i" # 1× uint64 + 14× int32 = 64 bytes
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT) # 64
SAMPLES_PER_PAGE = 4
PAGE_SIZE = SAMPLE_SIZE * SAMPLES_PER_PAGE # 256

CSV_HEADER = [
    "timestamp_ms",
    "pressure_kPa",
    "accel_x_ms2",
    "accel_y_ms2",
    "accel_z_ms2",
    "gyro_x_rads",
    "gyro_y_rads",
    "gyro_z_rads",
]


def sv_to_float(v1: int, v2: int) -> float:
    return v1 + v2 / 1_000_000


def parse_sample(data: bytes) -> dict:
    fields = struct.unpack(SAMPLE_FMT, data)
    ts = fields[0]
    (p1, p2,
     ax1, ax2, ay1, ay2, az1, az2,
     gx1, gx2, gy1, gy2, gz1, gz2) = fields[1:]

    return {
        "timestamp_ms": ts,
        "pressure_kPa": sv_to_float(p1, p2),
        "accel_x_ms2": sv_to_float(ax1, ax2),
        "accel_y_ms2": sv_to_float(ay1, ay2),
        "accel_z_ms2": sv_to_float(az1, az2),
        "gyro_x_rads": sv_to_float(gx1, gx2),
        "gyro_y_rads": sv_to_float(gy1, gy2),
        "gyro_z_rads": sv_to_float(gz1, gz2),
    }


def parse_bin(bin_path: Path, csv_path: Path) -> int:
    raw = bin_path.read_bytes()
    total = len(raw)
    if total % SAMPLE_SIZE != 0:
        print(f"Warning: file size {total} is not a multiple of {SAMPLE_SIZE} bytes; "
              f"trailing {total % SAMPLE_SIZE} bytes ignored.", file=sys.stderr)

    n_samples = total // SAMPLE_SIZE
    rows = []
    for i in range(n_samples):
        chunk = raw[i * SAMPLE_SIZE : (i + 1) * SAMPLE_SIZE]
        rows.append(parse_sample(chunk))

    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
        writer.writeheader()
        writer.writerows(rows)

    return n_samples


def main():
    # Default name
    if len(sys.argv) < 2:
        bin_path = Path("DATA.BIN")
    else:
        bin_path = Path(sys.argv[1])

    # Output path is input.csv if unspecified
    csv_path = Path(sys.argv[2]) if (len(sys.argv) >= 3) else (bin_path.with_suffix(".csv"))

    if not bin_path.exists():
        print(f"Error: {bin_path} not found.", file=sys.stderr)
        sys.exit(1)

    n = parse_bin(bin_path, csv_path)
    print(f"Wrote {n} samples ({n // SAMPLES_PER_PAGE} pages) -> {csv_path}")


if __name__ == "__main__":
    main()
