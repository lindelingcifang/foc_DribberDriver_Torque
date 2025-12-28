#!/usr/bin/env python3
"""Analyze lock-in (open-loop FOC) oscillation from CSV logs.

Input CSV expected columns:
Index,Time,debug_PWM_A,debug_PWM_B,debug_PWM_C,debug_IA,debug_IB,debug_IC,pos_estimate_debug

Outputs:
- Console summary of key metrics
- A PNG plot saved next to the CSV (stem + _lockin_osc.png)

Notes:
- Uses non-interactive matplotlib backend (Agg)
- Uses only ASCII labels to avoid font warnings
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib

matplotlib.use("Agg")
# Mute font lookup spam if system fonts are unusual
try:
    matplotlib.set_loglevel("error")
except Exception:
    logging.getLogger("matplotlib").setLevel(logging.ERROR)
    logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

import matplotlib.pyplot as plt


PWM_COLS = ["debug_PWM_A", "debug_PWM_B", "debug_PWM_C"]
I_COLS = ["debug_IA", "debug_IB", "debug_IC"]
T_COL = "Time"
POS_COL = "pos_estimate_debug"


def _to_numeric(df: pd.DataFrame, cols: list[str]) -> None:
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")


def clarke_abc_to_alphabeta(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Power-invariant Clarke transform."""
    sqrt3 = np.sqrt(3.0)
    alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c)
    beta = (2.0 / 3.0) * ((sqrt3 / 2.0) * b - (sqrt3 / 2.0) * c)
    return alpha, beta


def robust_sample_rate(t: np.ndarray) -> float:
    dt = np.diff(t)
    dt = dt[np.isfinite(dt)]
    dt = dt[dt > 0]
    if dt.size == 0:
        return float("nan")
    return 1.0 / np.median(dt)


def dominant_freq_fft(x: np.ndarray, fs: float) -> tuple[float, float]:
    """Return (f_peak_hz, peak_to_rms_ratio)."""
    x = x[np.isfinite(x)]
    if x.size < 64 or not np.isfinite(fs) or fs <= 0:
        return float("nan"), float("nan")

    x = x - np.mean(x)
    n = int(2 ** np.floor(np.log2(x.size)))
    if n < 64:
        return float("nan"), float("nan")
    x = x[:n]

    w = np.hanning(n)
    X = np.fft.rfft(x * w)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)

    mag = np.abs(X)
    if mag.size < 3:
        return float("nan"), float("nan")

    # Ignore DC
    mag[0] = 0.0
    k = int(np.argmax(mag))
    f_peak = float(freqs[k])

    rms = float(np.sqrt(np.mean(x**2)))
    peak = float(np.max(np.abs(x)))
    ratio = peak / (rms + 1e-12)
    return f_peak, ratio


def linear_fit_1d(t: np.ndarray, y: np.ndarray) -> tuple[float, float, float, float]:
    """Fit y ≈ m*t + b. Returns (m, b, r2, rmse)."""
    m, b = np.polyfit(t, y, 1)
    y_hat = m * t + b
    resid = y - y_hat
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((y - float(np.mean(y))) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rmse = float(np.sqrt(np.mean(resid**2)))
    return float(m), float(b), r2, rmse


def _event_times(t: np.ndarray, idx: np.ndarray) -> np.ndarray:
    idx = idx.astype(int)
    idx = idx[(idx >= 0) & (idx < t.size)]
    return t[idx]


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze lock-in oscillation from PWM/current/pos CSV.")
    parser.add_argument("csv", nargs="?", default="Debug/2.csv", help="Path to CSV (default: Debug/2.csv)")
    parser.add_argument("--vbus", type=float, default=12.0, help="Bus voltage in V (default: 12.0)")
    parser.add_argument("--inj-th", type=float, default=1e-4, help="Injection threshold on duty vector norm (default: 1e-4)")
    parser.add_argument("--ang-vmin", type=float, default=0.05, help="Min |V| (V) for reliable angle (default: 0.05)")
    parser.add_argument("--expected-omega", type=float, default=40.0, help="Expected electrical omega (rad/s), e.g. lockin vel (default: 40)")
    parser.add_argument("--pole-pairs", type=int, default=7, help="Motor pole pairs for omega_m = omega_e/p (default: 7)")
    parser.add_argument(
        "--omega-flip-min",
        type=float,
        default=5.0,
        help="Min |d(angle)/dt| (rad/s) to consider a flip event (default: 5)",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)
    required = [T_COL, *PWM_COLS, *I_COLS, POS_COL]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise SystemExit(f"Missing columns: {missing}. Found: {list(df.columns)}")

    _to_numeric(df, required)

    t = df[T_COL].to_numpy(dtype=float)
    fs = robust_sample_rate(t)

    # PWM to phase voltages around midpoint: duty 0.5 -> 0V
    duty_a = df[PWM_COLS[0]].to_numpy(dtype=float)
    duty_b = df[PWM_COLS[1]].to_numpy(dtype=float)
    duty_c = df[PWM_COLS[2]].to_numpy(dtype=float)

    va = (duty_a - 0.5) * args.vbus
    vb = (duty_b - 0.5) * args.vbus
    vc = (duty_c - 0.5) * args.vbus

    v_alpha, v_beta = clarke_abc_to_alphabeta(va, vb, vc)
    v_mag = np.sqrt(v_alpha**2 + v_beta**2)

    # Angle of voltage vector.
    # raw in [-pi, pi], unwrapped continuous (except when |V| ~ 0).
    v_ang_raw = np.arctan2(v_beta, v_alpha)
    v_ang_unwrap = np.unwrap(v_ang_raw)

    # Identify "injection" region where duty deviates from 0.5.
    # Treat (0,0,0) as inactive logging/default.
    dv_a = duty_a - 0.5
    dv_b = duty_b - 0.5
    dv_c = duty_c - 0.5
    inj_norm = np.sqrt(dv_a**2 + dv_b**2 + dv_c**2)
    inactive_mask = (
        np.isfinite(duty_a)
        & np.isfinite(duty_b)
        & np.isfinite(duty_c)
        & (duty_a == 0.0)
        & (duty_b == 0.0)
        & (duty_c == 0.0)
    )
    inj_mask = np.isfinite(inj_norm) & (~inactive_mask) & (inj_norm > args.inj_th)

    # Currents
    ia = df[I_COLS[0]].to_numpy(dtype=float)
    ib = df[I_COLS[1]].to_numpy(dtype=float)
    ic = df[I_COLS[2]].to_numpy(dtype=float)
    i_sum = ia + ib + ic
    i_alpha, i_beta = clarke_abc_to_alphabeta(ia, ib, ic)
    i_mag = np.sqrt(i_alpha**2 + i_beta**2)

    # Position and velocity
    pos = df[POS_COL].to_numpy(dtype=float)
    dt = np.diff(t)
    dpos = np.diff(pos)
    vel = np.full_like(pos, np.nan)
    with np.errstate(divide="ignore", invalid="ignore"):
        vel[1:] = dpos / dt

    # Summary prints
    t0 = float(np.nanmin(t))
    t1 = float(np.nanmax(t))
    print("=" * 60)
    print("Lock-in oscillation analysis")
    print(f"File: {csv_path}")
    print("=" * 60)
    print(f"Samples: {len(df)}")
    print(f"Time span: {t0:.6f}s .. {t1:.6f}s (T={t1 - t0:.3f}s)")
    print(f"Estimated sample rate: {fs:.2f} Hz")

    inj_count = int(np.sum(inj_mask))
    print("\n=== Injection window ===")
    print(f"Injected samples: {inj_count} ({inj_count/len(df)*100:.1f}%)")
    if inj_count > 0:
        t_inj0 = float(np.nanmin(t[inj_mask]))
        t_inj1 = float(np.nanmax(t[inj_mask]))
        print(f"Injected time span: {t_inj0:.6f}s .. {t_inj1:.6f}s (T={t_inj1 - t_inj0:.3f}s)")

    def _stats(name: str, x: np.ndarray, mask: np.ndarray | None = None) -> None:
        if mask is not None:
            x = x[mask]
        x = x[np.isfinite(x)]
        if x.size == 0:
            print(f"{name}: no valid samples")
            return
        print(f"{name}: min={np.min(x):.4f} max={np.max(x):.4f} mean={np.mean(x):.4f} rms={np.sqrt(np.mean(x**2)):.4f}")

    print("\n=== PWM-derived voltage (alpha-beta) ===")
    _stats("|V| [V]", v_mag, inj_mask)

    # Instantaneous electrical omega from unwrapped voltage angle
    omega_e_inst = np.full_like(v_ang_unwrap, np.nan)
    with np.errstate(divide="ignore", invalid="ignore"):
        omega_e_inst[1:] = np.diff(v_ang_unwrap) / dt

    # Angle quality gate: when |V| is too small, angle is noisy/meaningless.
    ang_good = inj_mask & np.isfinite(v_ang_unwrap) & np.isfinite(t) & np.isfinite(v_mag) & (v_mag >= args.ang_vmin)

    print("\n=== V_angle linear fit (electrical) ===")
    if int(np.sum(ang_good)) >= 20:
        m, b, r2, rmse = linear_fit_1d(t[ang_good], v_ang_unwrap[ang_good])
        print(f"Fit: angle ≈ omega_e*t + b")
        print(f"  omega_e_fit = {m:.4f} rad/s")
        print(f"  expected_omega = {args.expected_omega:.4f} rad/s")
        if np.isfinite(args.expected_omega) and args.expected_omega != 0:
            print(f"  ratio(fit/expected) = {m/args.expected_omega:.4f}")
        print(f"  R^2 = {r2:.4f}, RMSE = {rmse:.4f} rad")

        # Also estimate mechanical omega from encoder pos (treated as truth)
        omega_m_fit = m / float(args.pole_pairs) if args.pole_pairs else float("nan")
        print("\n=== pos linear fit (mechanical, encoder as truth) ===")
        pos_good = inj_mask & np.isfinite(pos) & np.isfinite(t)
        if int(np.sum(pos_good)) >= 20:
            mpos, bpos, r2pos, rmsepos = linear_fit_1d(t[pos_good], pos[pos_good])
            print(f"  omega_m_fit(pos) = {mpos:.6f} rad/s")
            if np.isfinite(omega_m_fit):
                print(f"  omega_m_from_omega_e = {omega_m_fit:.6f} rad/s (omega_e/p)")
            print(f"  R^2 = {r2pos:.4f}, RMSE = {rmsepos:.6f} rad")
    else:
        print("Not enough valid samples for angle fit (try lowering --ang-vmin).")

    # Wrap (reset) points: raw angle crosses +/-pi causing large discontinuity
    raw_diff = np.diff(v_ang_raw)
    wrap_idx = np.where(np.isfinite(raw_diff) & (np.abs(raw_diff) > np.pi))[0] + 1

    # Flip points: omega_e changes sign with sufficient magnitude
    om = omega_e_inst
    om_good = np.isfinite(om) & ang_good
    s = np.sign(om)
    s_prev = s[:-1]
    s_curr = s[1:]
    flip_candidates = np.where(
        om_good[1:]
        & om_good[:-1]
        & (s_prev != 0)
        & (s_curr != 0)
        & (s_prev != s_curr)
        & (np.abs(om[:-1]) >= args.omega_flip_min)
        & (np.abs(om[1:]) >= args.omega_flip_min)
    )[0] + 1

    print("\n=== Event markers ===")
    print(f"Wrap/reset events (raw angle jump > pi): {int(wrap_idx.size)}")
    if wrap_idx.size:
        times = _event_times(t, wrap_idx)
        preview = ", ".join([f"{x:.3f}" for x in times[:10]])
        print(f"  first events at t=[{preview}{' ...' if times.size > 10 else ''}] s")

    print(f"Flip events (sign change in omega_e): {int(flip_candidates.size)}")
    if flip_candidates.size:
        times = _event_times(t, flip_candidates)
        preview = ", ".join([f"{x:.3f}" for x in times[:10]])
        print(f"  first events at t=[{preview}{' ...' if times.size > 10 else ''}] s")

    print("\n=== Measured phase currents ===")
    _stats("|I| [A]", i_mag, inj_mask)
    _stats("I_sum=Ia+Ib+Ic [A]", i_sum, inj_mask)

    print("\n=== Position estimate ===")
    _stats("pos", pos, inj_mask)
    _stats("vel", vel, inj_mask)

    # Oscillation frequency estimate from position in injection window
    if inj_count > 0:
        pos_inj = pos[inj_mask]
        f_peak, ratio = dominant_freq_fft(pos_inj, fs)
        if np.isfinite(f_peak):
            print(f"Dominant pos frequency (FFT): {f_peak:.2f} Hz (peak/rms={ratio:.2f})")

    # Quick heuristics for likely causes
    print("\n=== Heuristic hints ===")
    # Current balance
    isum_inj = i_sum[inj_mask]
    isum_inj = isum_inj[np.isfinite(isum_inj)]
    if isum_inj.size:
        if np.sqrt(np.mean(isum_inj**2)) > 0.2:
            print("- I_sum RMS > 0.2A: current sensing offset / scaling mismatch is possible.")
        else:
            print("- I_sum near 0: phase current balance looks OK.")

    # Compare applied voltage vs current
    v_inj = v_mag[inj_mask]
    i_inj = i_mag[inj_mask]
    v_inj = v_inj[np.isfinite(v_inj)]
    i_inj = i_inj[np.isfinite(i_inj)]
    if v_inj.size and i_inj.size:
        v_rms = float(np.sqrt(np.mean(v_inj**2)))
        i_rms = float(np.sqrt(np.mean(i_inj**2)))
        if i_rms < 0.2 and v_rms > 0.5:
            print("- Voltage injected but current stays tiny: check motor wiring, inverter enable, or current units/logging.")
        elif i_rms > 3.0 and v_rms < 1.0:
            print("- Large current with small voltage: check current scaling / shunt gain config.")

    # Position change under injection
    if inj_count > 0:
        p = pos[inj_mask]
        p = p[np.isfinite(p)]
        if p.size:
            p2p = float(np.max(p) - np.min(p))
            print(f"- pos peak-to-peak (injection window): {p2p:.6f}")
            if p2p < 1e-3:
                print("- pos hardly changes while injecting: if you observe real motion, pos_estimate_debug may not be the mechanical angle (observer drift / different unit).")

    # Plot
    out_png = csv_path.parent / f"{csv_path.stem}_lockin_osc.png"

    fig, ax = plt.subplots(5, 1, figsize=(14, 12), sharex=True)

    ax[0].plot(t, duty_a, label="duty_a", linewidth=1)
    ax[0].plot(t, duty_b, label="duty_b", linewidth=1)
    ax[0].plot(t, duty_c, label="duty_c", linewidth=1)
    ax[0].axhline(0.5, color="k", linestyle="--", alpha=0.3)
    ax[0].set_ylabel("Duty")
    ax[0].legend(loc="upper right")
    ax[0].grid(True, alpha=0.3)

    ax[1].plot(t, v_mag, label="|V|", color="purple", linewidth=1.5)
    ax[1].set_ylabel("|V| (V)")
    ax[1].legend(loc="upper right")
    ax[1].grid(True, alpha=0.3)

    # Angle panel + fit
    ax[2].plot(t, v_ang_unwrap, label="V_angle_unwrap", color="tab:orange", linewidth=1.2)
    if int(np.sum(ang_good)) >= 20:
        m, b, r2, rmse = linear_fit_1d(t[ang_good], v_ang_unwrap[ang_good])
        ax[2].plot(t, m * t + b, label=f"fit omega_e={m:.2f} rad/s", color="k", linewidth=2, alpha=0.8)
    ax[2].set_ylabel("Angle (rad)")
    ax[2].legend(loc="upper right")
    ax[2].grid(True, alpha=0.3)

    ax[3].plot(t, ia, label="Ia", linewidth=1)
    ax[3].plot(t, ib, label="Ib", linewidth=1)
    ax[3].plot(t, ic, label="Ic", linewidth=1)
    ax[3].plot(t, i_sum, label="I_sum", color="k", linewidth=1, alpha=0.8)
    ax[3].set_ylabel("I (A)")
    ax[3].legend(loc="upper right")
    ax[3].grid(True, alpha=0.3)

    ax[4].plot(t, pos, label="pos_estimate", linewidth=1.2)
    ax[4].set_ylabel("pos (rad)")
    ax[4].set_xlabel("Time (s)")
    ax[4].legend(loc="upper right")
    ax[4].grid(True, alpha=0.3)

    # Shade injection window
    if inj_count > 0:
        ax[0].fill_between(t, 0, 1, where=inj_mask, color="orange", alpha=0.06, transform=ax[0].get_xaxis_transform())
        ax[1].fill_between(t, 0, 1, where=inj_mask, color="orange", alpha=0.06, transform=ax[1].get_xaxis_transform())
        ax[2].fill_between(t, 0, 1, where=inj_mask, color="orange", alpha=0.06, transform=ax[2].get_xaxis_transform())
        ax[3].fill_between(t, 0, 1, where=inj_mask, color="orange", alpha=0.06, transform=ax[3].get_xaxis_transform())
        ax[4].fill_between(t, 0, 1, where=inj_mask, color="orange", alpha=0.06, transform=ax[4].get_xaxis_transform())

    # Mark wrap/reset and flip events
    def _vlines(ax_list: list[plt.Axes], times: np.ndarray, color: str, alpha: float, lw: float, label: str | None = None) -> None:
        if times.size == 0:
            return
        first = True
        for tt in times:
            for a in ax_list:
                a.axvline(tt, color=color, alpha=alpha, linewidth=lw, label=(label if first else None))
            first = False

    wrap_t = _event_times(t, wrap_idx)
    flip_t = _event_times(t, flip_candidates)
    _vlines(list(ax), wrap_t, color="tab:red", alpha=0.25, lw=1.0, label="wrap/reset")
    _vlines(list(ax), flip_t, color="tab:blue", alpha=0.25, lw=1.0, label="flip")

    # If we added labels via vlines, show them in angle panel legend
    handles, labels = ax[2].get_legend_handles_labels()
    if handles and labels:
        ax[2].legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"\nSaved plot: {out_png}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
