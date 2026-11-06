"""Figure F02: PID Term Decomposition (P, I, and D contributions).

Shows:
  - Top panel: Representative error curve e(t) = r - y(t) during a step response
  - Three subpanels:
      1. Proportional term P(t) = Kp * e(t) (Present error, Blue)
      2. Integral term I(t) = Ki * integral(e dt) (Past accumulated error, Green)
      3. Derivative term D(t) = Kd * de/dt (Future trend / Braking rate, Red)
  - Bottom panel: Total combined control force u(t) = P + I + D
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from pidlab.plants import MassSpringDamper
from pidlab.pid import PID
from pidlab.sim import simulate
from pidlab.style import (
    setup_style,
    save_fig,
    COLOR_P,
    COLOR_I,
    COLOR_D,
    COLOR_SETPOINT,
    COLOR_OUTPUT,
    COLOR_FORCE,
)


def generate_pid_terms_figure(output_path: str = "figures/fig02_pid_terms.png"):
    setup_style()

    # Simulate realistic step response with PID(300, 300, 30)
    plant = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    controller = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, d_on="measurement")
    sim = simulate(plant, controller, setpoint=1.0, T=3.0, dt_sim=0.001)

    t = sim["t"]
    e = sim["r"] - sim["y"]
    p = sim["p"]
    i = sim["i"]
    d = sim["d"]
    u = sim["u"]

    fig = plt.figure(figsize=(7.5, 6.2))
    gs = fig.add_gridspec(3, 3, height_ratios=[1.1, 1.2, 1.1], hspace=0.38, wspace=0.28)

    # 1. Top Panel: Error curve e(t)
    ax_err = fig.add_subplot(gs[0, :])
    ax_err.plot(t, e, color="#444444", lw=1.8, label="Error $e(t) = r(t) - y(t)$")
    ax_err.axhline(0, color="k", linestyle=":", alpha=0.6)
    ax_err.set_ylabel("Error $e$ (m)")
    ax_err.set_title("System Error Trajectory $e(t)$ over Time", fontsize=10, weight="bold")
    ax_err.legend(loc="upper right")
    ax_err.set_xlim(0, 2.5)

    # 2. Middle 3 Panels: P, I, D terms
    # P-term
    ax_p = fig.add_subplot(gs[1, 0])
    ax_p.plot(t, p, color=COLOR_P, lw=1.8)
    ax_p.axhline(0, color="k", linestyle=":", alpha=0.5)
    ax_p.set_title("Proportional (P)\n$u_P = K_p \\cdot e(t)$", fontsize=9.5, weight="bold", color=COLOR_P)
    ax_p.set_ylabel("Force (N)")
    ax_p.set_xlabel("Time (s)")
    ax_p.set_xlim(0, 2.5)
    ax_p.text(0.05, 0.15, "Present:\nReacts to current gap", transform=ax_p.transAxes, fontsize=7.5,
              bbox=dict(boxstyle="round,pad=0.2", fc="#f0f6fa", ec=COLOR_P, alpha=0.8))

    # I-term
    ax_i = fig.add_subplot(gs[1, 1])
    ax_i.plot(t, i, color=COLOR_I, lw=1.8)
    ax_i.axhline(20.0, color="#1c6b1c", linestyle="--", lw=1.2, label="Holds 20 N")
    ax_i.axhline(0, color="k", linestyle=":", alpha=0.5)
    ax_i.set_title("Integral (I)\n$u_I = K_i \\int_0^t e(\\tau) d\\tau$", fontsize=9.5, weight="bold", color=COLOR_I)
    ax_i.set_xlabel("Time (s)")
    ax_i.set_xlim(0, 2.5)
    ax_i.legend(loc="lower right", fontsize=7.5)
    ax_i.text(0.05, 0.45, "Past:\nAccumulates memory,\nsettles at 20 N", transform=ax_i.transAxes, fontsize=7.5,
              bbox=dict(boxstyle="round,pad=0.2", fc="#f0faf0", ec=COLOR_I, alpha=0.8))

    # D-term
    ax_d = fig.add_subplot(gs[1, 2])
    ax_d.plot(t, d, color=COLOR_D, lw=1.8)
    ax_d.axhline(0, color="k", linestyle=":", alpha=0.5)
    ax_d.set_title("Derivative (D)\n$u_D = -K_d \\cdot \\dot{y}(t)$", fontsize=9.5, weight="bold", color=COLOR_D)
    ax_d.set_xlabel("Time (s)")
    ax_d.set_xlim(0, 2.5)
    ax_d.text(0.05, 0.15, "Trend / Braking:\nOpposes high velocity,\ndamps oscillation", transform=ax_d.transAxes, fontsize=7.5,
              bbox=dict(boxstyle="round,pad=0.2", fc="#faf0f0", ec=COLOR_D, alpha=0.8))

    # 3. Bottom Panel: Total Actuator Force u(t) = P + I + D
    ax_u = fig.add_subplot(gs[2, :])
    ax_u.plot(t, p, color=COLOR_P, lw=1.2, linestyle="--", alpha=0.7, label="$u_P$ (Present)")
    ax_u.plot(t, i, color=COLOR_I, lw=1.2, linestyle="--", alpha=0.7, label="$u_I$ (Past)")
    ax_u.plot(t, d, color=COLOR_D, lw=1.2, linestyle="--", alpha=0.7, label="$u_D$ (Trend)")
    ax_u.plot(t, u, color=COLOR_OUTPUT, lw=2.0, label="Total Force $u(t) = P + I + D$")
    ax_u.axhline(20.0, color="#1c6b1c", linestyle=":", alpha=0.8, label="Holding Force (20 N)")
    ax_u.set_xlabel("Time (s)")
    ax_u.set_ylabel("Applied Force (N)")
    ax_u.set_title("Total Control Force $u(t)$ and Component Summation", fontsize=10, weight="bold")
    ax_u.legend(loc="upper right", ncol=3, fontsize=8)
    ax_u.set_xlim(0, 2.5)

    save_fig(fig, output_path)
    print(f"Generated: {output_path}")


if __name__ == "__main__":
    generate_pid_terms_figure()
