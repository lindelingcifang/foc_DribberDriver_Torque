#!/usr/bin/env python3
"""Analyze closed-loop control oscillation in 4.csv.

Expected scenario:
- Motor enters closed-loop at ~32s
- Light disturbance at ~37s triggers violent oscillation

Focus:
- Position/velocity/current behavior before/during/after disturbance
- Oscillation frequency (structural resonance vs control instability)
- Phase relationships (position vs torque)
- Gain margin indicators

Outputs:
- Console diagnostics
- Multi-panel time-domain plot
- Frequency analysis
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal

import matplotlib

matplotlib.use("Agg")
try:
    matplotlib.set_loglevel("error")
except Exception:
    logging.getLogger("matplotlib").setLevel(logging.ERROR)
    logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

import matplotlib.pyplot as plt


def robust_derivative(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Compute dy/dt with same length as y (forward diff + repeat last)."""
    dt = np.diff(t)
    dy = np.diff(y)
    deriv = np.full_like(y, np.nan)
    with np.errstate(divide="ignore", invalid="ignore"):
        deriv[:-1] = dy / dt
    deriv[-1] = deriv[-2] if len(deriv) > 1 else np.nan
    return deriv


def analyze_oscillation_fft(t: np.ndarray, y: np.ndarray, fs: float, label: str = "signal") -> None:
    """Print FFT peak analysis."""
    y = y[np.isfinite(y)]
    if len(y) < 100:
        print(f"  {label}: insufficient data for FFT")
        return

    # Welch PSD
    nperseg = min(512, len(y) // 2)
    freqs, psd = signal.welch(y, fs=fs, nperseg=nperseg)

    # Find peaks
    peak_idx = np.argmax(psd[1:]) + 1  # skip DC
    peak_freq = freqs[peak_idx]
    peak_power = psd[peak_idx]

    print(f"  {label}:")
    print(f"    dominant freq: {peak_freq:.2f} Hz")
    print(f"    peak PSD: {peak_power:.3e}")
    print(f"    rms: {np.sqrt(np.mean(y**2)):.6f}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Closed-loop oscillation analysis for 4.csv")
    ap.add_argument("csv", nargs="?", default="Debug/4.csv")
    ap.add_argument("--t-closedloop", type=float, default=32.0, help="Time when closed-loop starts")
    ap.add_argument("--t-disturbance", type=float, default=37.0, help="Time of disturbance")
    ap.add_argument("--pole-pairs", type=int, default=7)
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        raise SystemExit(f"CSV not found: {path}")

    df = pd.read_csv(path)

    # Coerce all to numeric
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")

    t = df["Time"].to_numpy(dtype=float)
    pos = df["pos_estimate_debug"].to_numpy(dtype=float)
    ia = df["debug_IA"].to_numpy(dtype=float)
    ib = df["debug_IB"].to_numpy(dtype=float)
    ic = df["debug_IC"].to_numpy(dtype=float)
    alpha = df.get("alpha_debug", pd.Series(np.nan, index=df.index)).to_numpy(dtype=float)
    beta = df.get("beta_debug", pd.Series(np.nan, index=df.index)).to_numpy(dtype=float)

    # Derived signals
    i_mag = np.sqrt(ia**2 + ib**2 + ic**2)
    vel = robust_derivative(pos, t)
    theta_e = pos * float(args.pole_pairs)

    fs = 1.0 / np.median(np.diff(t[np.isfinite(t)]))

    print("=" * 70)
    print("Closed-loop oscillation analysis")
    print(f"File: {path}")
    print("=" * 70)
    print(f"Samples: {len(df)}")
    print(f"Time span: {float(np.nanmin(t)):.3f}s .. {float(np.nanmax(t)):.3f}s")
    print(f"Sample rate: {fs:.2f} Hz")

    # Define regions
    t_cl = args.t_closedloop
    t_dist = args.t_disturbance

    mask_before = (t >= 0) & (t < t_cl)
    mask_steady = (t >= t_cl) & (t < t_dist)
    mask_osc = t >= t_dist

    for name, mask in [("Before CL", mask_before), ("Steady CL", mask_steady), ("Post-disturbance", mask_osc)]:
        n = int(np.sum(mask))
        if n == 0:
            continue
        pos_seg = pos[mask]
        vel_seg = vel[mask]
        i_seg = i_mag[mask]

        print(f"\n{name} ({n} samples):")
        print(f"  pos: mean={float(np.nanmean(pos_seg)):.6f} std={float(np.nanstd(pos_seg)):.6f} range={float(np.nanmax(pos_seg) - np.nanmin(pos_seg)):.6f}")
        print(f"  vel: rms={float(np.sqrt(np.nanmean(vel_seg**2))):.6f} std={float(np.nanstd(vel_seg)):.6f}")
        print(f"  current: mean={float(np.nanmean(i_seg)):.3f}A rms={float(np.sqrt(np.nanmean(i_seg**2))):.3f}A max={float(np.nanmax(i_seg)):.3f}A")

    # FFT analysis on post-disturbance region
    if int(np.sum(mask_osc)) > 200:
        print(f"\nOscillation frequency analysis (t >= {t_dist}s):")
        t_osc = t[mask_osc]
        pos_osc = pos[mask_osc]
        vel_osc = vel[mask_osc]
        i_osc = i_mag[mask_osc]

        analyze_oscillation_fft(t_osc, pos_osc, fs, "position")
        analyze_oscillation_fft(t_osc, vel_osc, fs, "velocity")
        analyze_oscillation_fft(t_osc, i_osc, fs, "current")

    # Phase analysis: position vs current (indicative of controller fighting itself)
    if int(np.sum(mask_osc)) > 100:
        pos_norm = (pos[mask_osc] - np.nanmean(pos[mask_osc])) / (np.nanstd(pos[mask_osc]) + 1e-12)
        i_norm = (i_mag[mask_osc] - np.nanmean(i_mag[mask_osc])) / (np.nanstd(i_mag[mask_osc]) + 1e-12)
        corr = float(np.corrcoef(pos_norm[np.isfinite(pos_norm) & np.isfinite(i_norm)],
                                  i_norm[np.isfinite(pos_norm) & np.isfinite(i_norm)])[0, 1])
        print(f"\nPosition-current correlation (post-disturbance): {corr:.3f}")
        if corr > 0.5:
            print("  NOTE: Strong positive correlation suggests controller is actively driving oscillation")
        elif corr < -0.5:
            print("  NOTE: Strong negative correlation (expected for damping)")

    # Plot
    out_png = path.parent / f"{path.stem}_oscillation.png"

    fig, ax = plt.subplots(5, 1, figsize=(16, 12), sharex=True)

    # Position
    ax[0].plot(t, pos, label="pos_estimate", linewidth=1.0)
    ax[0].axvline(t_cl, color="green", linestyle="--", alpha=0.7, label=f"CL start ({t_cl}s)")
    ax[0].axvline(t_dist, color="red", linestyle="--", alpha=0.7, label=f"Disturbance ({t_dist}s)")
    ax[0].set_ylabel("Position (rad)")
    ax[0].legend(loc="upper right")
    ax[0].grid(True, alpha=0.3)

    # Velocity
    ax[1].plot(t, vel, label="d(pos)/dt", linewidth=1.0, color="C1")
    ax[1].axvline(t_cl, color="green", linestyle="--", alpha=0.7)
    ax[1].axvline(t_dist, color="red", linestyle="--", alpha=0.7)
    ax[1].set_ylabel("Velocity (rad/s)")
    ax[1].legend(loc="upper right")
    ax[1].grid(True, alpha=0.3)

    # Currents
    ax[2].plot(t, ia, label="IA", linewidth=0.8, alpha=0.7)
    ax[2].plot(t, ib, label="IB", linewidth=0.8, alpha=0.7)
    ax[2].plot(t, ic, label="IC", linewidth=0.8, alpha=0.7)
    ax[2].plot(t, i_mag, label="|I|", linewidth=1.2, color="black")
    ax[2].axvline(t_cl, color="green", linestyle="--", alpha=0.7)
    ax[2].axvline(t_dist, color="red", linestyle="--", alpha=0.7)
    ax[2].set_ylabel("Current (A)")
    ax[2].legend(loc="upper right", ncol=2)
    ax[2].grid(True, alpha=0.3)

    # Alpha/Beta (voltage command)
    ax[3].plot(t, alpha, label="alpha (Vα)", linewidth=1.0)
    ax[3].plot(t, beta, label="beta (Vβ)", linewidth=1.0)
    v_mag = np.sqrt(alpha**2 + beta**2)
    ax[3].plot(t, v_mag, label="|V|", linewidth=1.2, color="black", alpha=0.6)
    ax[3].axvline(t_cl, color="green", linestyle="--", alpha=0.7)
    ax[3].axvline(t_dist, color="red", linestyle="--", alpha=0.7)
    ax[3].set_ylabel("Voltage (V)")
    ax[3].legend(loc="upper right")
    ax[3].grid(True, alpha=0.3)

    # Electrical angle (for rotor tracking check)
    ax[4].plot(t, np.unwrap(theta_e), label="theta_e (unwrapped)", linewidth=1.0, color="C4")
    ax[4].axvline(t_cl, color="green", linestyle="--", alpha=0.7)
    ax[4].axvline(t_dist, color="red", linestyle="--", alpha=0.7)
    ax[4].set_ylabel("Elec. angle (rad)")
    ax[4].set_xlabel("Time (s)")
    ax[4].legend(loc="upper right")
    ax[4].grid(True, alpha=0.3)

    # Shade post-disturbance region
    for a in ax:
        a.fill_betweenx([a.get_ylim()[0], a.get_ylim()[1]], t_dist, float(np.nanmax(t)),
                        color="red", alpha=0.05)

    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"\nSaved plot: {out_png}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
