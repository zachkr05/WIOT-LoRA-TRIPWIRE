#!/usr/bin/env python3
"""
tripwire_gp.py — GP presence detection via RSSI anomaly.

Phase 1 (first 30s): Collect quiet RSSI data, fit a GP per link.
Phase 2 (forever):   Predict RSSI from time, flag anomaly if
                     |observed - predicted| > 3.4 dB.

Usage:
    python tripwire_gp.py --port /dev/cu.usbserial-0001
    python tripwire_gp.py --pkl data/tripwire_run_20260426_144757.pkl

Dependencies:
    pip install pyserial pandas numpy
"""

import argparse
import math
import time
from typing import Optional

import numpy as np


# ═══════════════════════════════════════════════════════════════════
# GP with Matérn 3/2 kernel
# ═══════════════════════════════════════════════════════════════════

def matern32_kernel(X1, X2, lengthscale, variance):
    """k(r) = σ²(1 + √3 r/l) exp(−√3 r/l)"""
    r = np.abs(X1 - X2.T)
    s = math.sqrt(3.0) * r / lengthscale
    return variance * (1.0 + s) * np.exp(-s)


class GP:
    """GP trained once on quiet data, then used for prediction."""

    def __init__(self, lengthscale: float = 10.0,
                 signal_var: float = 2.0,
                 noise_var: float = 0.5):
        self.ls = lengthscale
        self.sv = signal_var
        self.nv = noise_var
        self.trained = False
        self.X_train = None
        self.L = None           # cholesky of K
        self.alpha = None       # L^T \ (L \ y)

    def train(self, times: list[float], rssi: list[float]):
        """Fit GP on quiet calibration data."""
        self.X_train = np.array(times).reshape(-1, 1)
        y = np.array(rssi)

        K = matern32_kernel(self.X_train, self.X_train, self.ls, self.sv)
        K += self.nv * np.eye(len(y))

        self.L = np.linalg.cholesky(K)
        self.alpha = np.linalg.solve(self.L.T, np.linalg.solve(self.L, y))
        self.trained = True

        # Report fit quality
        mu_train = np.squeeze(
            matern32_kernel(self.X_train, self.X_train, self.ls, self.sv) @ self.alpha
        )
        residuals = y - mu_train
        print(f"    GP trained on {len(y)} samples")
        print(f"    RSSI range: [{y.min():.1f}, {y.max():.1f}] dBm")
        print(f"    Train residual std: {residuals.std():.2f} dB")

    def predict(self, t: float) -> tuple[float, float]:
        """Return (mu, std) at time t."""
        if not self.trained:
            return 0.0, 999.0

        ts = np.array([[t]])
        k_s = matern32_kernel(self.X_train, ts, self.ls, self.sv)
        k_ss = self.sv + self.nv

        v = np.linalg.solve(self.L, k_s)
        mu = float(np.squeeze(k_s.T @ self.alpha))
        var = max(float(np.squeeze(k_ss - v.T @ v)), 1e-6)

        return mu, math.sqrt(var)


# ═══════════════════════════════════════════════════════════════════
# PIPELINE
# ═══════════════════════════════════════════════════════════════════

RESIDUAL_THRESHOLD = 3.4  # dB — flag if |observed - predicted| exceeds this
CALIBRATION_SECS = 30.0


class TripwireGP:

    def __init__(self):
        self.gp1 = GP()
        self.gp2 = GP()
        self.t0: Optional[float] = None

        # Calibration buffers
        self._cal_t1: list[float] = []
        self._cal_v1: list[float] = []
        self._cal_t2: list[float] = []
        self._cal_v2: list[float] = []
        self._calibrated = False

    def _rel(self, wall: float) -> float:
        if self.t0 is None:
            self.t0 = wall
        return wall - self.t0

    def _process(self, wall: float, sender: str, rssi: float):
        t = self._rel(wall)

        # ── Phase 1: collect calibration data ─────────────────
        if not self._calibrated:
            if sender == "S1":
                self._cal_t1.append(t)
                self._cal_v1.append(rssi)
            else:
                self._cal_t2.append(t)
                self._cal_v2.append(rssi)

            elapsed = t
            count = len(self._cal_t1) + len(self._cal_t2)
            # Progress every 5s
            if count % 20 == 0:
                print(f"  [calibrating] {elapsed:.0f}/{CALIBRATION_SECS:.0f}s  "
                      f"S1={len(self._cal_t1)} S2={len(self._cal_t2)} samples")

            if elapsed >= CALIBRATION_SECS:
                print(f"\n{'='*55}")
                print(f"  CALIBRATION COMPLETE — fitting GPs")
                print(f"{'='*55}")
                print(f"  S1:")
                self.gp1.train(self._cal_t1, self._cal_v1)
                print(f"  S2:")
                self.gp2.train(self._cal_t2, self._cal_v2)
                print(f"{'='*55}")
                print(f"  Threshold: |residual| > {RESIDUAL_THRESHOLD} dB = anomaly")
                print(f"  Monitoring started.\n")
                self._calibrated = True
            return

        # ── Phase 2: predict and detect ───────────────────────
        gp = self.gp1 if sender == "S1" else self.gp2
        mu, std = gp.predict(t)
        residual = rssi - mu
        anomaly = abs(residual) > RESIDUAL_THRESHOLD

        flag = " *** PERSON DETECTED" if anomaly else ""
        print(f"{sender}  rssi={rssi:6.1f}  "
              f"predicted={mu:6.1f}  "
              f"residual={residual:+5.1f}  "
              f"gp_std={std:4.1f}{flag}")

    # ── Data sources ──────────────────────────────────────────

    def run_serial(self, port: str, baud: int = 115200):
        import serial as pyserial
        ser = pyserial.Serial(port, baud, timeout=1)
        print(f"[tripwire_gp] Connected to {port}")
        print(f"[tripwire_gp] Keep corridor CLEAR for {CALIBRATION_SECS:.0f}s…\n")

        try:
            while True:
                try:
                    raw = ser.readline()
                except (pyserial.SerialException, OSError):
                    time.sleep(0.05)
                    continue
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line.startswith("DATA,"):
                    continue
                parts = line.split(",")
                if len(parts) < 6:
                    continue
                try:
                    sender = parts[2]
                    rssi = float(parts[4])
                except (ValueError, IndexError):
                    continue

                self._process(time.time(), sender, rssi)

        except KeyboardInterrupt:
            print("\n[tripwire_gp] Stopped.")
        finally:
            ser.close()

    def run_pkl(self, path: str):
        import pandas as pd
        df = pd.read_pickle(path)
        print(f"[offline] {len(df)} rows from {path}")
        print(f"[offline] First {CALIBRATION_SECS:.0f}s used for calibration.\n")

        for _, r in df.iterrows():
            self._process(r["wall_time"], r["sender"], r["filt_rssi"])


def main():
    ap = argparse.ArgumentParser(description="GP tripwire — presence detection")
    ap.add_argument("--port", default="/dev/cu.usbserial-0001")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--pkl", help="Run offline on collected .pkl")
    args = ap.parse_args()

    pipe = TripwireGP()
    if args.pkl:
        pipe.run_pkl(args.pkl)
    else:
        pipe.run_serial(args.port, args.baud)


if __name__ == "__main__":
    main()