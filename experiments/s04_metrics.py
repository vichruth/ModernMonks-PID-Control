"""Figure F04: Step Response Performance Criteria Annotation.

Explicitly illustrates and defines the four key time-domain performance metrics:
  1. Rise Time (t_r): Duration from 10% to 90% of target setpoint.
  2. Peak Overshoot (M_p): Maximum percentage deviation above setpoint: (y_max - r) / r * 100%.
  3. Settling Time (t_s): Time after which response remains permanently within +/- 2% error band.
  4. Steady-State Error (e_ss): Remaining asymptotic gap between target setpoint and final value.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from pidlab.plants import MassSpringDamper
from pidlab.pid import PID
from pidlab.sim import simulate
from pidlab.metrics import calc_metrics
from pidlab.style import (
    setup_style,
    save_fig,
    COLOR_OUTPUT,
    COLOR_SETPOINT,
    COLOR_P,
    COLOR_D,
    COLOR_BAND,
)


def generate_metrics_figure(output_path: str = "figures/fig04_step_metrics.png"):
    setup_style()

    # Controller with clear, clean overshoot and settling dynamics for textbook illustration
    plant = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    controller = PID(kp=220.0, ki=180.0, kd=14.0, dt=0.001)
    sim = simulate(plant, controller, setpoint=1.0, T=3.2, dt_sim=0.001)

    t = sim["t"]
    y = sim["y"]
    r = sim["r"]

    m = calc_metrics(t, y, r)
    peak_val = m["peak_y"]
    peak_idx = int(np.argmax(y))
    t_peak = t[peak_idx]

    # Calculate 10% and 90% crossing times
    idx_10 = np.where(y >= 0.10)[0][0]
    idx_90 = np.where(y >= 0.90)[0][0]
    t_10 = t[idx_10]
    t_90 = t[idx_90]
    t_settle = m["settling_time"]

    fig, ax = plt.subplots(figsize=(7.8, 5.0))

    # 1. Settling Band (+/- 2% of setpoint)
    ax.axhspan(0.98, 1.02, color="#e6f2e6", alpha=0.9, label=r"$\pm 2\%$ Settling Band ($0.98 \mathrm{m} - 1.02 \mathrm{m}$)")
    ax.axhline(1.02, color="#2ca02c", linestyle=":", lw=1.1, alpha=0.7)
    ax.axhline(0.98, color="#2ca02c", linestyle=":", lw=1.1, alpha=0.7)

    # 2. Setpoint and 10%/90% guide lines
    ax.axhline(1.0, color=COLOR_SETPOINT, linestyle="--", lw=1.5, label="Target Setpoint $r = 1.0$ m")
    ax.axhline(0.90, color="#888888", linestyle=":", lw=1.0, alpha=0.6)
    ax.axhline(0.10, color="#888888", linestyle=":", lw=1.0, alpha=0.6)

    # 3. Simulated Response Curve
    ax.plot(t, y, color=COLOR_OUTPUT, lw=2.2, label="System Response $y(t)$")

    # -------------------------------------------------------------
    # ANNOTATION 1: Rise Time (10% to 90%)
    # -------------------------------------------------------------
    ax.vlines([t_10, t_90], 0, [0.10, 0.90], color=COLOR_P, linestyle="--", lw=1.2)
    ax.annotate(
        "", xy=(t_90, 0.50), xytext=(t_10, 0.50),
        arrowprops=dict(arrowstyle="<->", color=COLOR_P, lw=1.6)
    )
    ax.text((t_10 + t_90) / 2, 0.54, f"Rise Time $t_r$\n({t_90 - t_10:.3f} s)",
            ha="center", va="bottom", color=COLOR_P, fontsize=8.5, weight="bold")

    # -------------------------------------------------------------
    # ANNOTATION 2: Peak Overshoot
    # -------------------------------------------------------------
    ax.plot(t_peak, peak_val, "ro", markersize=6)
    ax.annotate(
        "", xy=(t_peak, peak_val), xytext=(t_peak, 1.0),
        arrowprops=dict(arrowstyle="<->", color=COLOR_D, lw=1.6)
    )
    os_pct = m["overshoot"]
    ax.text(t_peak + 0.05, (peak_val + 1.0) / 2,
            f"Overshoot $M_p = {os_pct:.1f}\\%$\nPeak = {peak_val:.3f} m",
            ha="left", va="center", color=COLOR_D, fontsize=8.5, weight="bold")

    # -------------------------------------------------------------
    # ANNOTATION 3: Settling Time (2% band)
    # -------------------------------------------------------------
    ax.axvline(t_settle, color="#1c6b1c", linestyle="--", lw=1.4)
    ax.annotate(
        f"Settling Time $t_s = {t_settle:.3f}$ s\n(Stays within $\\pm 2\\%$ band)",
        xy=(t_settle, 1.02), xytext=(t_settle + 0.35, 1.15),
        arrowprops=dict(arrowstyle="->", color="#1c6b1c", lw=1.4),
        ha="left", va="bottom", color="#1c6b1c", fontsize=8.5, weight="bold",
        bbox=dict(boxstyle="round,pad=0.25", fc="#ffffff", ec="#1c6b1c", alpha=0.9)
    )

    # -------------------------------------------------------------
    # ANNOTATION 4: Steady-State Error
    # -------------------------------------------------------------
    ax.text(2.6, 0.94, r"Steady-State Error $e_{ss} = r - y(\infty) \to 0$",
            fontsize=8.5, color="#333333", weight="bold",
            bbox=dict(boxstyle="square,pad=0.25", fc="#fbfbfb", ec="#999999", lw=0.8))

    ax.set_title("Figure 4. Step Response Specification Criteria (Rise, Overshoot, Settling, Steady Error)",
                 fontsize=10.5, weight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Displacement $y(t)$ (m)")
    ax.set_xlim(0, 3.2)
    ax.set_ylim(-0.05, 1.32)
    ax.legend(loc="lower right", fontsize=8.5)

    save_fig(fig, output_path)
    print(f"Generated: {output_path}")


if __name__ == "__main__":
    generate_metrics_figure()
