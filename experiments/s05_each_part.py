"""Section 5: Testing Each PID Component in Isolation.

Generates:
  - Figure F05: P-only step responses across gains (Kp = 10, 50, 100, 300, 1000)
  - Figure F06: Theoretical vs simulated steady-state error k/(Kp+k) and damping/overshoot
  - Figure F07: Adding Integral (I) action (Kp = 100, Ki = 0, 50, 200, 500) and integral force settling at 20 N
  - Figure F08: Adding Derivative (D) action (Kp = 300, Kd = 0, 10, 30, 60) showing suppression of oscillation
  - Table T01: Performance metrics for P-only controller sweep
  - Table T02: Analytical theory vs simulation benchmark comparison
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from pidlab.plants import MassSpringDamper
from pidlab.pid import PID
from pidlab.sim import simulate
from pidlab.metrics import calc_metrics, format_metrics
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


def run_section_5():
    setup_style()
    os.makedirs("figures", exist_ok=True)
    os.makedirs("results", exist_ok=True)

    # =========================================================================
    # PART 1: Proportional (P) Only
    # =========================================================================
    kp_list = [10.0, 50.0, 100.0, 300.0, 1000.0]
    p_colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    p_results = []

    fig_f05, (ax_p_pos, ax_p_force) = plt.subplots(
        2, 1, figsize=(7.5, 5.5), sharex=True, gridspec_kw={"height_ratios": [1.4, 0.9]}
    )
    ax_p_pos.axhline(1.0, color=COLOR_SETPOINT, linestyle="--", lw=1.5, label="Target (1.0 m)")

    for kp, col in zip(kp_list, p_colors):
        plant = MassSpringDamper(m=1.0, c=10.0, k=20.0)
        ctrl = PID(kp=kp, ki=0.0, kd=0.0, dt=0.001)
        sim = simulate(plant, ctrl, setpoint=1.0, T=3.0, dt_sim=0.001)
        m = calc_metrics(sim["t"], sim["y"], sim["r"], u=sim["u"])

        theory_ess = 20.0 / (kp + 20.0)
        theory_xss = kp / (kp + 20.0)

        # Closed-loop damping ratio: m*x'' + c*x' + (k+Kp)*x = Kp*r
        omega_n = np.sqrt((20.0 + kp) / 1.0)
        zeta = 10.0 / (2.0 * omega_n)
        if zeta < 1.0:
            os_factor = np.exp(-np.pi * zeta / np.sqrt(1.0 - zeta**2))
            theory_os = ((theory_xss * (1.0 + os_factor) - 1.0) / 1.0) * 100.0
            theory_os = max(0.0, theory_os)
        else:
            theory_os = 0.0

        p_results.append({
            "kp": kp,
            "sim_ess": m["steady_state_error"],
            "theory_ess": theory_ess,
            "sim_os": m["overshoot"],
            "theory_os": theory_os,
            "zeta": zeta,
            "metrics": m,
        })

        lbl = f"$K_p = {int(kp)}$ ($e_{{ss}}={m['steady_state_error']:.3f}$m, OS={m['overshoot']:.1f}%)"
        ax_p_pos.plot(sim["t"], sim["y"], color=col, lw=1.8, label=lbl)
        ax_p_pos.axhline(theory_xss, color=col, linestyle=":", alpha=0.4)
        ax_p_force.plot(sim["t"], sim["u"], color=col, lw=1.6)

    ax_p_pos.set_ylabel("Displacement $x(t)$ (m)")
    ax_p_pos.set_title("Figure 5. Proportional-Only Control: Effect of Gain $K_p$ on Steady Error and Oscillation",
                       fontsize=10.5, weight="bold")
    ax_p_pos.legend(loc="lower right", fontsize=8)
    ax_p_pos.set_xlim(0, 3.0)
    ax_p_pos.set_ylim(-0.05, 1.65)

    ax_p_force.set_xlabel("Time (s)")
    ax_p_force.set_ylabel("Control Force $u(t)$ (N)")
    ax_p_force.axhline(20.0, color="#2ca02c", linestyle=":", lw=1.2, label="Required 20 N Holding Force")
    ax_p_force.legend(loc="upper right", fontsize=8)

    save_fig(fig_f05, "figures/fig05_p_only.png")
    print("Generated: figures/fig05_p_only.png")

    # =========================================================================
    # PART 2: Figure F06 - Theory vs Simulation Validation Curves
    # =========================================================================
    kp_fine = np.logspace(0.8, 3.2, 200)
    theory_ess_curve = 20.0 / (kp_fine + 20.0)

    theory_os_curve = []
    for kp in kp_fine:
        x_ss = kp / (20.0 + kp)
        wn = np.sqrt(20.0 + kp)
        z = 10.0 / (2.0 * wn)
        if z < 1.0:
            peak = x_ss * (1.0 + np.exp(-np.pi * z / np.sqrt(1.0 - z**2)))
            theory_os_curve.append(max(0.0, (peak - 1.0) * 100.0))
        else:
            theory_os_curve.append(0.0)
    theory_os_curve = np.array(theory_os_curve)

    fig_f06, (ax_ess, ax_os) = plt.subplots(1, 2, figsize=(8.0, 3.8))

    ax_ess.plot(kp_fine, theory_ess_curve, "b-", lw=2.0, label=r"Theory: $e_{ss} = \frac{k}{K_p + k}$")
    sim_kps = [r["kp"] for r in p_results]
    sim_errors = [r["sim_ess"] for r in p_results]
    ax_ess.plot(sim_kps, sim_errors, "ro", markersize=6, label="Simulated Dots")
    ax_ess.set_xscale("log")
    ax_ess.set_xlabel("Proportional Gain $K_p$ (N/m, log scale)")
    ax_ess.set_ylabel("Steady-State Error $e_{ss}$ (m)")
    ax_ess.set_title("Steady-State Error vs Gain $K_p$", fontsize=9.5, weight="bold")
    ax_ess.legend(loc="upper right", fontsize=8.5)
    ax_ess.set_ylim(-0.02, 0.72)

    ax_os.plot(kp_fine, theory_os_curve, "r-", lw=2.0, label="Analytical Overshoot vs Setpoint")
    sim_overshoots = [r["sim_os"] for r in p_results]
    ax_os.plot(sim_kps, sim_overshoots, "bo", markersize=6, label="Simulated Dots")
    ax_os.set_xscale("log")
    ax_os.set_xlabel("Proportional Gain $K_p$ (N/m, log scale)")
    ax_os.set_ylabel("Overshoot (%)")
    ax_os.set_title("Overshoot vs Gain $K_p$ (Damping Drop)", fontsize=9.5, weight="bold")
    ax_os.legend(loc="upper left", fontsize=8.5)

    fig_f06.suptitle("Figure 6. Theory vs Simulation: Steady-State Offset & Closed-Loop Damping Loss",
                     fontsize=10.5, weight="bold", y=1.02)
    save_fig(fig_f06, "figures/fig06_p_theory_vs_sim.png")
    print("Generated: figures/fig06_p_theory_vs_sim.png")

    # =========================================================================
    # PART 3: Figure F07 - Adding Integral (I) Action
    # =========================================================================
    ki_list = [0.0, 50.0, 200.0, 500.0]
    i_colors = ["#7f7f7f", "#1f77b4", "#2ca02c", "#d62728"]

    fig_f07, (ax_i_pos, ax_i_term) = plt.subplots(
        2, 1, figsize=(7.5, 5.5), sharex=True, gridspec_kw={"height_ratios": [1.3, 1.0]}
    )
    ax_i_pos.axhline(1.0, color=COLOR_SETPOINT, linestyle="--", lw=1.5, label="Target (1.0 m)")

    for ki, col in zip(ki_list, i_colors):
        plant = MassSpringDamper(m=1.0, c=10.0, k=20.0)
        ctrl = PID(kp=100.0, ki=ki, kd=0.0, dt=0.001)
        sim = simulate(plant, ctrl, setpoint=1.0, T=4.0, dt_sim=0.001)
        m = calc_metrics(sim["t"], sim["y"], sim["r"])

        lbl = f"$K_i = {int(ki)}$ ($e_{{ss}}={m['steady_state_error']:.3f}$m, OS={m['overshoot']:.1f}%)"
        ax_i_pos.plot(sim["t"], sim["y"], color=col, lw=1.8, label=lbl)
        ax_i_term.plot(sim["t"], sim["i"], color=col, lw=1.8, label=f"$u_I$ ($K_i={int(ki)}$)")

    ax_i_pos.set_ylabel("Displacement $x(t)$ (m)")
    ax_i_pos.set_title("Figure 7. Adding Integral Action ($K_p = 100$): Error Annihilation and Holding Memory",
                       fontsize=10.5, weight="bold")
    ax_i_pos.legend(loc="lower right", fontsize=8.5)
    ax_i_pos.set_xlim(0, 4.0)
    ax_i_pos.set_ylim(-0.05, 1.45)

    ax_i_term.set_xlabel("Time (s)")
    ax_i_term.set_ylabel("Integral Term $u_I(t)$ (N)")
    ax_i_term.axhline(20.0, color="#1c6b1c", linestyle=":", lw=1.5, label="Target Holding Force = 20 N")
    ax_i_term.text(0.15, 21.0, "All stable I-loops settle at exactly 20 N to cancel spring force at 1 m",
                   color="#1c6b1c", fontsize=8.0, weight="bold")
    ax_i_term.legend(loc="lower right", fontsize=8.5)
    ax_i_term.set_ylim(-2, 35)

    save_fig(fig_f07, "figures/fig07_add_i.png")
    print("Generated: figures/fig07_add_i.png")

    # =========================================================================
    # PART 4: Figure F08 - Adding Derivative (D) Action
    # =========================================================================
    kd_list = [0.0, 10.0, 30.0, 60.0]
    d_colors = ["#d62728", "#ff7f0e", "#2ca02c", "#1f77b4"]

    fig_f08, (ax_d_pos, ax_d_force) = plt.subplots(
        2, 1, figsize=(7.5, 5.5), sharex=True, gridspec_kw={"height_ratios": [1.4, 0.9]}
    )
    ax_d_pos.axhline(1.0, color=COLOR_SETPOINT, linestyle="--", lw=1.5, label="Target (1.0 m)")

    for kd, col in zip(kd_list, d_colors):
        plant = MassSpringDamper(m=1.0, c=10.0, k=20.0)
        ctrl = PID(kp=300.0, ki=0.0, kd=kd, dt=0.001, d_on="measurement")
        sim = simulate(plant, ctrl, setpoint=1.0, T=2.5, dt_sim=0.001)
        m = calc_metrics(sim["t"], sim["y"], sim["r"], u=sim["u"])

        # Closed-loop damping ratio: zeta = (c + Kd) / (2 * sqrt(m * (k + Kp)))
        zeta = (10.0 + kd) / (2.0 * np.sqrt(320.0))

        lbl = f"$K_d = {int(kd)}$ ($\\zeta = {zeta:.2f}$, OS={m['overshoot']:.1f}%)"
        ax_d_pos.plot(sim["t"], sim["y"], color=col, lw=1.8, label=lbl)
        ax_d_force.plot(sim["t"], sim["u"], color=col, lw=1.6, label=f"$K_d = {int(kd)}$")

    ax_d_pos.set_ylabel("Displacement $x(t)$ (m)")
    ax_d_pos.set_title("Figure 8. Adding Derivative Action ($K_p = 300$): Dynamic Braking & Oscillation Quenching",
                       fontsize=10.5, weight="bold")
    ax_d_pos.legend(loc="lower right", fontsize=8.5)
    ax_d_pos.set_xlim(0, 2.5)
    ax_d_pos.set_ylim(-0.05, 1.45)

    ax_d_pos.text(
        0.04, 0.65,
        r"Damping Ratio: $\zeta = \frac{c + K_d}{2\sqrt{m(k + K_p)}}$" "\n"
        r"• Critical damping $\zeta = 1.0$ requires $K_d \approx 25.8$ Ns/m" "\n"
        r"• $K_d = 30$ gives $\zeta = 1.12 > 1$ (Overdamped: 0% overshoot)",
        transform=ax_d_pos.transAxes, fontsize=8.0,
        bbox=dict(boxstyle="round,pad=0.3", fc="#ffffff", ec="#cccccc", lw=0.8)
    )

    ax_d_force.set_xlabel("Time (s)")
    ax_d_force.set_ylabel("Applied Force $u(t)$ (N)")
    ax_d_force.legend(loc="upper right", fontsize=8.5)

    save_fig(fig_f08, "figures/fig08_add_d.png")
    print("Generated: figures/fig08_add_d.png")

    # =========================================================================
    # PART 5: Tables T01 and T02 Generation
    # =========================================================================
    t01_lines = [
        "# Table 1: Proportional-Only Controller Performance Sweep",
        "",
        "Plant parameters: $m = 1.0$ kg, $c = 10.0$ Ns/m, $k = 20.0$ N/m. Setpoint $r = 1.0$ m.",
        "",
        "| $K_p$ (N/m) | Steady-State Error $e_{ss}$ | Final Value $x_{ss}$ | Overshoot (%) | Rise Time (s) | Settling Time (s) | Peak Force (N) |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in p_results:
        m = r["metrics"]
        fm = format_metrics(m)
        t01_lines.append(
            f"| {int(r['kp'])} | {fm['steady_state_error']} | {1.0 - m['steady_state_error']:.4f} m | {fm['overshoot']} | {fm['rise_time']} | {fm['settling_time']} | {fm['peak_force']} |"
        )
    with open("results/t01_p_metrics.md", "w") as f:
        f.write("\n".join(t01_lines) + "\n")
    print("Saved: results/t01_p_metrics.md")

    t02_lines = [
        "# Table 2: Analytical Theory vs Numerical Simulation Benchmark",
        "",
        r"Comparison of steady-state error $e_{ss} = \frac{k}{K_p + k}$ and damping ratio $\zeta = \frac{c}{2\sqrt{m(k+K_p)}}$.",
        "",
        r"| $K_p$ | Theory $e_{ss}$ | Sim $e_{ss}$ | Error Diff | Damping $\zeta$ | Theory Overshoot | Sim Overshoot |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in p_results:
        diff = abs(r["sim_ess"] - r["theory_ess"])
        t02_lines.append(
            f"| {int(r['kp'])} | {r['theory_ess']:.4f} | {r['sim_ess']:.4f} | {diff:.6f} | {r['zeta']:.3f} | {r['theory_os']:.2f}% | {r['sim_os']:.2f}% |"
        )
    with open("results/t02_theory_vs_sim.md", "w") as f:
        f.write("\n".join(t02_lines) + "\n")
    print("Saved: results/t02_theory_vs_sim.md")


if __name__ == "__main__":
    run_section_5()

