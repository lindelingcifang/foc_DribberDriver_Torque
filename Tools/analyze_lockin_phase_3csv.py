#!/usr/bin/env python3
"""Phase-focused analysis for richer lock-in logs (3.csv).

Expected columns:
Index,Time,debug_PWM_A,debug_PWM_B,debug_PWM_C,
debug_IA,debug_IB,debug_IC,
pos_estimate_debug,
open_phase_debug,open_phase_vel_debug,
open_Vd_debug,open_Vq_debug,
alpha_debug,beta_debug,
sectant_debug

Focus:
- Phase alignment among open_phase, alpha/beta vector angle, and encoder pos (treated as truth)
- Consistency of open_phase_vel with d(open_phase)/dt and with d(angle(alpha,beta))/dt
- Consistency of alpha/beta with inverse Park(open_Vd, open_Vq, open_phase)

Outputs:
- Console summary
- Plot saved as <stem>_phase.png

Notes:
- Uses Agg backend and suppresses font spam.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib

matplotlib.use("Agg")
try:
    matplotlib.set_loglevel("error")
except Exception:
    logging.getLogger("matplotlib").setLevel(logging.ERROR)
    logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

import matplotlib.pyplot as plt


COLS = {
    "t": "Time",
    "pos": "pos_estimate_debug",
    "open_phase": "open_phase_debug",
    "open_phase_vel": "open_phase_vel_debug",
    "vd": "open_Vd_debug",
    "vq": "open_Vq_debug",
    "alpha": "alpha_debug",
    "beta": "beta_debug",
    "sect": "sectant_debug",
}


def wrap_pm_pi(x: np.ndarray) -> np.ndarray:
    return (x + np.pi) % (2 * np.pi) - np.pi


def robust_sample_rate(t: np.ndarray) -> float:
    dt = np.diff(t)
    dt = dt[np.isfinite(dt)]
    dt = dt[dt > 0]
    if dt.size == 0:
        return float("nan")
    return 1.0 / np.median(dt)


def linear_fit_1d(t: np.ndarray, y: np.ndarray) -> tuple[float, float, float, float]:
    m, b = np.polyfit(t, y, 1)
    y_hat = m * t + b
    resid = y - y_hat
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((y - float(np.mean(y))) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rmse = float(np.sqrt(np.mean(resid**2)))
    return float(m), float(b), r2, rmse


def stats_line(name: str, x: np.ndarray) -> str:
    x = x[np.isfinite(x)]
    if x.size == 0:
        return f"{name}: no valid samples"
    return (
        f"{name}: mean={float(np.mean(x)):.6f} std={float(np.std(x)):.6f} "
        f"min={float(np.min(x)):.6f} max={float(np.max(x)):.6f} rms={float(np.sqrt(np.mean(x**2))):.6f}"
    )


def fit_global_scale(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """Fit y ~ k*x (least squares) and return (k, rmse)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    den = float(np.dot(x, x))
    k = float(np.dot(x, y) / den) if den > 1e-12 else 0.0
    rmse = float(np.sqrt(np.mean((y - k * x) ** 2)))
    return k, rmse


def corrcoef_safe(x: np.ndarray, y: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if x.size < 2:
        return float("nan")
    if not (np.isfinite(np.std(x)) and np.isfinite(np.std(y))):
        return float("nan")
    if float(np.std(x)) < 1e-12 or float(np.std(y)) < 1e-12:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def sector_from_angle(angle_rad: np.ndarray) -> np.ndarray:
    """Return SVPWM sector index in 1..6 from electrical angle.

    Uses 60-degree bins on angle mapped to [0, 2pi).
    """
    a = (angle_rad + np.pi) % (2 * np.pi)  # [0, 2pi)
    return (np.floor(a / (np.pi / 3)).astype(int) + 1)


def evaluate_inverse_park_candidates(
    open_phase: np.ndarray,
    vd: np.ndarray,
    vq: np.ndarray,
    alpha_meas: np.ndarray,
    beta_meas: np.ndarray,
    mask: np.ndarray,
) -> list[dict]:
    """Rank candidate dq->alpha/beta models.

    This is meant to catch common implementation bugs (e.g. using cos() where sin() should be).
    Returns a list of dicts sorted by RMSE (lower is better).
    """
    ph = open_phase[mask]
    vd = vd[mask]
    vq = vq[mask]
    a = alpha_meas[mask]
    b = beta_meas[mask]

    s = np.sin(ph)
    c = np.cos(ph)

    # Candidate models: (alpha_pred, beta_pred)
    cands: dict[str, tuple[np.ndarray, np.ndarray]] = {
        "correct": (vd * c - vq * s, vd * s + vq * c),
        "swap_sc": (vd * s - vq * c, vd * c + vq * s),
        # common bug: beta uses cos instead of sin (or vice versa)
        "sin_eq_cos": (vd * c - vq * c, vd * c + vq * c),
        "cos_eq_sin": (vd * s - vq * s, vd * s + vq * s),
        "neg_sin": (vd * c + vq * s, -vd * s + vq * c),
        "neg_cos": (-vd * c - vq * s, vd * s - vq * c),
    }

    rows: list[dict] = []
    y = np.concatenate([a, b])
    for name, (ap, bp) in cands.items():
        x = np.concatenate([ap, bp])
        k, rmse = fit_global_scale(x, y)
        rows.append(
            {
                "name": name,
                "k": k,
                "rmse": rmse,
                "corr_alpha": corrcoef_safe(ap, a),
                "corr_beta": corrcoef_safe(bp, b),
                "alpha_pred": k * ap,
                "beta_pred": k * bp,
            }
        )

    rows.sort(key=lambda d: float(d["rmse"]))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description="Phase-focused analysis for 3.csv")
    ap.add_argument("csv", nargs="?", default="Debug/3.csv")
    ap.add_argument("--pole-pairs", type=int, default=7)
    ap.add_argument("--vmin", type=float, default=0.05, help="Min |alpha,beta| for reliable angle")
    ap.add_argument("--expected-omega", type=float, default=40.0, help="Expected electrical omega (rad/s)")
    ap.add_argument("--fit-t0", type=float, default=None, help="Optional fit window start time")
    ap.add_argument("--fit-t1", type=float, default=None, help="Optional fit window end time")
    args = ap.parse_args()

    path = Path(args.csv)
    if not path.exists():
        raise SystemExit(f"CSV not found: {path}")

    df = pd.read_csv(path)
    missing = [c for c in COLS.values() if c not in df.columns]
    if missing:
        raise SystemExit(f"Missing columns: {missing}. Found: {list(df.columns)}")

    for c in COLS.values():
        df[c] = pd.to_numeric(df[c], errors="coerce")

    t = df[COLS["t"]].to_numpy(dtype=float)
    pos = df[COLS["pos"]].to_numpy(dtype=float)
    open_phase = df[COLS["open_phase"]].to_numpy(dtype=float)
    open_phase_vel = df[COLS["open_phase_vel"]].to_numpy(dtype=float)
    vd = df[COLS["vd"]].to_numpy(dtype=float)
    vq = df[COLS["vq"]].to_numpy(dtype=float)
    alpha = df[COLS["alpha"]].to_numpy(dtype=float)
    beta = df[COLS["beta"]].to_numpy(dtype=float)
    sect = df[COLS["sect"]].to_numpy(dtype=float)

    fs = robust_sample_rate(t)
    dt = np.diff(t)

    ab_mag = np.sqrt(alpha**2 + beta**2)
    ab_ang_raw = np.arctan2(beta, alpha)
    ab_ang_unwrap = np.unwrap(ab_ang_raw)

    # Determine active window: any of these becoming non-zero
    active_mask = (
        np.isfinite(open_phase)
        & np.isfinite(alpha)
        & np.isfinite(beta)
        & (
            (np.abs(open_phase) > 1e-9)
            | (np.abs(open_phase_vel) > 1e-9)
            | (ab_mag > 1e-6)
            | (np.abs(vd) > 1e-9)
            | (np.abs(vq) > 1e-9)
        )
    )

    # Angle quality gate
    ang_good = active_mask & np.isfinite(ab_ang_unwrap) & np.isfinite(t) & (ab_mag >= args.vmin)

    # Optional fit window
    fit_mask = ang_good.copy()
    if args.fit_t0 is not None:
        fit_mask &= t >= args.fit_t0
    if args.fit_t1 is not None:
        fit_mask &= t <= args.fit_t1

    # omega from open_phase derivative
    omega_from_phase = np.full_like(open_phase, np.nan)
    with np.errstate(divide="ignore", invalid="ignore"):
        omega_from_phase[1:] = np.diff(np.unwrap(open_phase)) / dt

    omega_from_ab = np.full_like(ab_ang_unwrap, np.nan)
    with np.errstate(divide="ignore", invalid="ignore"):
        omega_from_ab[1:] = np.diff(ab_ang_unwrap) / dt

    # Electrical angle from encoder pos (truth)
    theta_e_from_pos = pos * float(args.pole_pairs)
    theta_e_from_pos_unwrap = np.unwrap(theta_e_from_pos)

    # Phase errors (wrapped)
    phase_err_open_vs_ab = wrap_pm_pi(open_phase - ab_ang_raw)
    phase_err_open_vs_pos = wrap_pm_pi(open_phase - wrap_pm_pi(theta_e_from_pos))

    # Inverse Park prediction: [alpha; beta] = [cos -sin; sin cos] * [vd; vq]
    c = np.cos(open_phase)
    s = np.sin(open_phase)
    alpha_pred = vd * c - vq * s
    beta_pred = vd * s + vq * c

    alpha_err = alpha - alpha_pred
    beta_err = beta - beta_pred

    # Also compare angles: if vq ~ 0 and vd > 0, ab_ang should match open_phase.
    # If vd < 0, angle shifts by pi.
    ab_ang_from_open = np.arctan2(beta_pred, alpha_pred)
    phase_err_pred_vs_meas = wrap_pm_pi(ab_ang_from_open - ab_ang_raw)

    print("=" * 70)
    print("3.csv phase analysis")
    print(f"File: {path}")
    print("=" * 70)
    print(f"Samples: {len(df)}")
    print(f"Time span: {float(np.nanmin(t)):.6f}s .. {float(np.nanmax(t)):.6f}s")
    print(f"Estimated sample rate: {fs:.2f} Hz")

    active_idx = np.where(active_mask)[0]
    if active_idx.size:
        t0 = float(t[active_idx[0]])
        t1 = float(t[active_idx[-1]])
        print(f"Active span: {t0:.6f}s .. {t1:.6f}s (T={t1 - t0:.3f}s)")

    print("\n=== Angle/phase core checks ===")
    good_n = int(np.sum(ang_good))
    print(f"Angle-good samples: {good_n} (|alpha,beta| >= {args.vmin})")

    # Quick structural diagnostics: alpha ~ beta implies broken dq->ab mapping
    abm_good = ang_good & np.isfinite(alpha) & np.isfinite(beta)
    if int(np.sum(abm_good)) >= 50:
        corr_ab = corrcoef_safe(alpha[abm_good], beta[abm_good])
        std_a = float(np.std(alpha[abm_good]))
        std_a_minus_b = float(np.std((alpha - beta)[abm_good]))
        print("\nAlpha/Beta structure")
        print(f"  corr(alpha, beta) = {corr_ab:.6f}")
        if std_a > 1e-12:
            print(f"  std(alpha-beta) / std(alpha) = {std_a_minus_b / std_a:.6f}")
        else:
            print(f"  std(alpha-beta) = {std_a_minus_b:.6e}")
        if np.isfinite(corr_ab) and corr_ab > 0.999:
            print("  NOTE: alpha and beta are almost identical -> ab vector cannot rotate normally.")
            print("        This strongly suggests a dq->ab inverse-Park implementation issue (e.g. sin/cos bug).")

    # Fit omega on ab angle
    if int(np.sum(fit_mask)) >= 20:
        m, b0, r2, rmse = linear_fit_1d(t[fit_mask], ab_ang_unwrap[fit_mask])
        print("\nElectrical omega from alpha/beta angle fit")
        print(f"  omega_e_fit = {m:.4f} rad/s")
        print(f"  expected_omega = {args.expected_omega:.4f} rad/s")
        if args.expected_omega:
            print(f"  ratio(fit/expected) = {m/args.expected_omega:.4f}")
        print(f"  R^2 = {r2:.4f}, RMSE = {rmse:.4f} rad")
    else:
        print("\nElectrical omega fit: not enough valid samples (adjust --vmin / --fit-t0/--fit-t1)")

    # Compare omega signals
    om_mask = ang_good & np.isfinite(omega_from_ab) & np.isfinite(open_phase_vel)
    if int(np.sum(om_mask)) >= 20:
        diff_omega = omega_from_ab - open_phase_vel
        print("\nOmega consistency")
        print(stats_line("  omega_from_ab - open_phase_vel", diff_omega[om_mask]))
        print(stats_line("  open_phase_vel", open_phase_vel[om_mask]))
        print(stats_line("  omega_from_ab", omega_from_ab[om_mask]))

    # open_phase derivative vs reported vel
    om2_mask = active_mask & np.isfinite(omega_from_phase) & np.isfinite(open_phase_vel)
    if int(np.sum(om2_mask)) >= 20:
        diff_omega2 = omega_from_phase - open_phase_vel
        print(stats_line("  d(open_phase)/dt - open_phase_vel", diff_omega2[om2_mask]))

    # Phase errors
    pe_mask = ang_good & np.isfinite(phase_err_open_vs_ab)
    if int(np.sum(pe_mask)) >= 20:
        print("\nPhase error")
        print(stats_line("  wrap(open_phase - angle(alpha,beta))", phase_err_open_vs_ab[pe_mask]))

    pp_mask = active_mask & np.isfinite(phase_err_open_vs_pos)
    if int(np.sum(pp_mask)) >= 20:
        print(stats_line("  wrap(open_phase - pole_pairs*pos)", phase_err_open_vs_pos[pp_mask]))

    # alpha/beta vs inverse Park prediction
    abm = active_mask & np.isfinite(alpha_err) & np.isfinite(beta_err)
    if int(np.sum(abm)) >= 20:
        print("\nInverse Park consistency (measured alpha/beta vs predicted from open_Vd/Vq + open_phase)")
        print(stats_line("  alpha_err", alpha_err[abm]))
        print(stats_line("  beta_err", beta_err[abm]))
        pem = active_mask & np.isfinite(phase_err_pred_vs_meas) & (ab_mag >= args.vmin)
        if int(np.sum(pem)) >= 20:
            print(stats_line("  wrap(angle(pred_alpha,beta) - angle(meas_alpha,beta))", phase_err_pred_vs_meas[pem]))

    # Candidate scoring: try to explain measured (alpha,beta) from (vd,vq,open_phase)
    cand_mask = active_mask & np.isfinite(open_phase) & np.isfinite(vd) & np.isfinite(vq) & np.isfinite(alpha) & np.isfinite(beta) & (ab_mag >= args.vmin)
    best = None
    ranked = []
    if int(np.sum(cand_mask)) >= 50:
        ranked = evaluate_inverse_park_candidates(open_phase, vd, vq, alpha, beta, cand_mask)
        best = ranked[0]
        print("\nInverse Park candidate ranking (lower RMSE is better)")
        for r in ranked[:5]:
            print(
                f"  {r['name']:<10s} rmse={float(r['rmse']):.6f} k={float(r['k']): .4f} "
                f"corr(alpha)={float(r['corr_alpha']): .3f} corr(beta)={float(r['corr_beta']): .3f}"
            )

    # Sectant sanity check vs atan2(beta,alpha)
    sect_mask = cand_mask & np.isfinite(sect)
    if int(np.sum(sect_mask)) >= 50:
        s_meas = np.round(sect[sect_mask]).astype(int)
        s_pred = sector_from_angle(ab_ang_raw[sect_mask])
        # Also try 180-deg shifted prediction (sector +3 modulo 6)
        s_pred_pi = ((s_pred + 2) % 6) + 1  # +3 with 1..6 indexing
        match = float(np.mean(s_meas == s_pred))
        match_pi = float(np.mean(s_meas == s_pred_pi))
        print("\nSectant consistency")
        print(f"  sectant_debug unique: {sorted(set(s_meas.tolist()))}")
        print(f"  match(sector(angle(alpha,beta))) = {match:.3f}")
        print(f"  match(sector(angle(alpha,beta)+pi)) = {match_pi:.3f}")
        if match_pi > match + 0.2:
            print("  NOTE: sectant_debug looks ~pi shifted relative to atan2(beta,alpha).")

    # Plot
    out_png = path.parent / f"{path.stem}_phase.png"

    fig, ax = plt.subplots(6, 1, figsize=(14, 14), sharex=True)

    ax[0].plot(t, pos, label="pos (mech rad)", linewidth=1.2)
    ax[0].plot(t, wrap_pm_pi(theta_e_from_pos), label="pos*p (wrapped, elec rad)", linewidth=1.0, alpha=0.8)
    ax[0].set_ylabel("pos")
    ax[0].legend(loc="upper right")
    ax[0].grid(True, alpha=0.3)

    ax[1].plot(t, wrap_pm_pi(open_phase), label="open_phase (wrapped)", linewidth=1.2)
    ax[1].plot(t, wrap_pm_pi(ab_ang_raw), label="angle(alpha,beta) (wrapped)", linewidth=1.0, alpha=0.9)
    ax[1].set_ylabel("angle (rad)")
    ax[1].legend(loc="upper right")
    ax[1].grid(True, alpha=0.3)

    ax[2].plot(t, phase_err_open_vs_ab, label="wrap(open_phase - ab_angle)", linewidth=1.0)
    ax[2].plot(t, phase_err_open_vs_pos, label="wrap(open_phase - pos*p)", linewidth=1.0, alpha=0.9)
    ax[2].set_ylabel("phase err (rad)")
    ax[2].legend(loc="upper right")
    ax[2].grid(True, alpha=0.3)

    ax[3].plot(t, open_phase_vel, label="open_phase_vel", linewidth=1.2)
    ax[3].plot(t, omega_from_ab, label="d(ab_angle)/dt", linewidth=1.0, alpha=0.9)
    ax[3].plot(t, omega_from_phase, label="d(open_phase)/dt", linewidth=1.0, alpha=0.7)
    ax[3].set_ylabel("omega (rad/s)")
    ax[3].legend(loc="upper right")
    ax[3].grid(True, alpha=0.3)

    ax[4].plot(t, vd, label="open_Vd", linewidth=1.2)
    ax[4].plot(t, vq, label="open_Vq", linewidth=1.2)
    ax[4].set_ylabel("Vdq")
    ax[4].legend(loc="upper right")
    ax[4].grid(True, alpha=0.3)

    ax[5].plot(t, alpha, label="alpha_meas", linewidth=1.0)
    ax[5].plot(t, beta, label="beta_meas", linewidth=1.0)
    if best is not None:
        # Write best candidate back into full-length arrays for plotting
        alpha_best = np.full_like(alpha, np.nan)
        beta_best = np.full_like(beta, np.nan)
        alpha_best[cand_mask] = best["alpha_pred"]
        beta_best[cand_mask] = best["beta_pred"]
        ax[5].plot(t, alpha_best, label=f"alpha_pred({best['name']})", linewidth=1.0, alpha=0.8)
        ax[5].plot(t, beta_best, label=f"beta_pred({best['name']})", linewidth=1.0, alpha=0.8)
    else:
        ax[5].plot(t, alpha_pred, label="alpha_pred(correct)", linewidth=1.0, alpha=0.8)
        ax[5].plot(t, beta_pred, label="beta_pred(correct)", linewidth=1.0, alpha=0.8)
    ax[5].plot(t, sect, label="sectant", linewidth=1.0, alpha=0.6)
    ax[5].set_ylabel("alpha/beta")
    ax[5].set_xlabel("Time (s)")
    ax[5].legend(loc="upper right")
    ax[5].grid(True, alpha=0.3)

    # Mark angle-good window
    if good_n > 0:
        for a in ax:
            a.fill_between(t, 0, 1, where=ang_good, color="orange", alpha=0.05, transform=a.get_xaxis_transform())

    fig.tight_layout()
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"\nSaved plot: {out_png}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
