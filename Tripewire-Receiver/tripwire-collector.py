#!/usr/bin/env python3
"""
tripwire_collector.py — Serial listener for the LoRa tripwire receiver.

Reads structured DATA lines from the receiver's serial port, adds wall-clock
timestamps, and saves everything to a pickle file. Also dumps CSV alongside
for quick inspection.

Usage:
    python tripwire_collector.py [--port /dev/cu.usbserial-0001] [--out data/run1]

Press Ctrl+C to stop — data is saved on exit and every --save-interval seconds.

Dependencies:
    pip install pyserial pandas
"""

import argparse
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import pandas as pd
import serial


# ── Schema matching the receiver's DATA line ──────────────────────
COLUMNS = [
    "wall_time",        # float, time.time() on host
    "wall_iso",         # str,   ISO 8601 for readability
    "rx_millis",        # int,   receiver's millis()
    "sender",           # str,   "S1" or "S2"
    "seq",              # int,   sender's packet sequence number
    "raw_rssi",         # float, dBm
    "filt_rssi",        # float, median-filtered RSSI
    "baseline",         # float, EMA baseline
    "delta",            # float, baseline - filt_rssi
    "snr",              # float, dB
]


def parse_data_line(line: str, wall_time: float) -> dict | None:
    """Parse a 'DATA,...' line into a dict, or None if malformed."""
    line = line.strip()
    if not line.startswith("DATA,"):
        return None
    parts = line.split(",")
    if len(parts) != 9:  # DATA + 8 fields
        return None
    try:
        return {
            "wall_time":  wall_time,
            "wall_iso":   datetime.fromtimestamp(wall_time).isoformat(),
            "rx_millis":  int(parts[1]),
            "sender":     parts[2],
            "seq":        int(parts[3]),
            "raw_rssi":   float(parts[4]),
            "filt_rssi":  float(parts[5]),
            "baseline":   float(parts[6]),
            "delta":      float(parts[7]),
            "snr":        float(parts[8]),
        }
    except (ValueError, IndexError):
        return None


def save_data(records: list[dict], out_stem: str):
    """Save to both .pkl and .csv."""
    if not records:
        print("[save] No records to save.")
        return
    df = pd.DataFrame(records, columns=COLUMNS)
    pkl_path = f"{out_stem}.pkl"
    csv_path = f"{out_stem}.csv"
    df.to_pickle(pkl_path)
    df.to_csv(csv_path, index=False)
    print(f"[save] {len(df)} records -> {pkl_path}  ({csv_path})")


def main():
    parser = argparse.ArgumentParser(description="LoRa tripwire data collector")
    parser.add_argument("--port", default="/dev/cu.usbserial-0001",
                        help="Serial port for the receiver")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="data/tripwire_run",
                        help="Output file stem (no extension)")
    parser.add_argument("--save-interval", type=int, default=30,
                        help="Auto-save every N seconds")
    args = parser.parse_args()

    # Ensure output directory exists
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    # Append timestamp to filename so runs don't overwrite
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_stem = f"{args.out}_{ts}"

    print(f"[collector] port={args.port}  baud={args.baud}")
    print(f"[collector] output stem: {out_stem}")
    print(f"[collector] auto-save every {args.save_interval}s")
    print(f"[collector] Press Ctrl+C to stop.\n")

    records: list[dict] = []
    last_save = time.time()
    running = True

    def shutdown(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"[error] Cannot open {args.port}: {e}")
        sys.exit(1)

    print(f"[collector] Connected to {args.port}. Listening...\n")

    s1_count = 0
    s2_count = 0

    try:
        while running:
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError):
                # USB serial buffer overrun on macOS — not data loss, just retry
                time.sleep(0.05)
                continue

            if not raw:
                continue

            wall = time.time()
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue

            # Always print the raw line for live monitoring
            print(line)

            # Parse DATA lines
            rec = parse_data_line(line, wall)
            if rec:
                records.append(rec)
                if rec["sender"] == "S1":
                    s1_count += 1
                else:
                    s2_count += 1

                # Compact status every 50 packets
                total = s1_count + s2_count
                if total % 50 == 0:
                    print(f"  --- {total} packets  (S1={s1_count}, S2={s2_count}) ---")

            # Periodic save
            now = time.time()
            if now - last_save >= args.save_interval:
                save_data(records, out_stem)
                last_save = now

    finally:
        ser.close()
        save_data(records, out_stem)
        print(f"\n[collector] Done. Total: {len(records)} records "
              f"(S1={s1_count}, S2={s2_count})")


if __name__ == "__main__":
    main()