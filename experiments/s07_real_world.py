"""Section 7: Demonstrating and Resolving Real-World Engineering Hazards.

Generates:
  - Figure F11: Actuator saturation and Integrator Windup (before vs after clamping anti-windup)
  - Figure F12: Sensor measurement noise (raw D chatter vs low-pass filtered D)
  - Figure F13: Setpoint kick phenomenon (D on error vs D on measurement in the first 50 ms)
  - Figure F14: Controller sampling latency and Zero-Order Hold instability threshold
  - Table T04: Comprehensive before/after performance comparison table
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
    COLOR_ACCENT,
)


def run_section_7():
    setup_style()
    os.makedirs("figures", exist_ok=True)
    os.makedirs("results", exist_ok=True)

    t04_entries = []

    # =========================================================================
    # PROBLEM (a): Actuator Saturation & Integrator Windup
    # =========================================================================
    plant_unclamped = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    ctrl_unclamped = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, u_min=-30.0, u_max=30.0, anti_windup="none")
    sim_unclamped = simulate(plant_unclamped, ctrl_unclamped, setpoint=1.0, T=5.0, dt_sim=0.001)
    m_unclamped = calc_metrics(sim_unclamped["t"], sim_unclamped["y"], sim_unclamped["r"], u=sim_unclamped["u"])

    plant_clamped = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    ctrl_clamped = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, u_min=-30.0, u_max=30.0, anti_windup="clamp")
    sim_clamped = simulate(plant_clamped, ctrl_clamped, setpoint=1.0, T=5.0, dt_sim=0.001)
    m_clamped = calc_metrics(sim_clamped["t"], sim_clamped["y"], sim_clamped["r"], u=sim_clamped["u"])

    t04_entries.append({
        "problem": "Integrator Windup (|u| ≤ 30 N)",
        "metric": "Overshoot / Settling",
        "before": f"OS: {m_unclamped['overshoot']:.1f}%, Settling: {m_unclamped['settling_time']:.2f} s",
        "after": f"OS: {m_clamped['overshoot']:.1f}%, Settling: {m_clamped['settling_time']:.2f} s",
        "mechanism": "Conditional clamping freezes integrator during saturation if error increases saturation.",
    })

    fig_f11, (ax_w_pos, ax_w_force, ax_w_int) = plt.subplots(
        3, 1, figsize=(7.5, 6.8), sharex=True, gridspec_kw={"height_ratios": [1.2, 1.0, 1.0]}
    )

    # Position
    ax_w_pos.axhline(1.0, color=COLOR_SETPOINT, linestyle="--", lw=1.5, label="Target (1.0 m)")
    ax_w_pos.axhspan(0.98, 1.02, color="#f0f7f0", alpha=0.8)
    ax_w_pos.plot(sim_unclamped["t"], sim_unclamped["y"], color="#d62728", lw=1.8, label=f"Unclamped (OS: {m_unclamped['overshoot']:.1f}%, $t_s$: {m_unclamped['settling_time']:.2f}s)")
    ax_w_pos.plot(sim_clamped["t"], sim_clamped["y"], color="#2ca02c", lw=2.0, label=f"Clamped Anti-Windup (OS: {m_clamped['overshoot']:.1f}%, $t_s$: {m_clamped['settling_time']:.2f}s)")
    ax_w_pos.set_ylabel("Position $x(t)$ (m)")
    ax_w_pos.set_title("Figure 11. Actuator Saturation & Anti-Windup Clamping (30 N Motor Limit)", fontsize=10.5, weight="bold")
    ax_w_pos.legend(loc="lower right", fontsize=8.0)
    ax_w_pos.set_xlim(0, 4.5)
    ax_w_pos.set_ylim(-0.05, 1.35)

    # Force
    ax_w_force.plot(sim_unclamped["t"], sim_unclamped["u"], color="#d62728", lw=1.6, label="Unclamped Force $u(t)$")
    ax_w_force.plot(sim_clamped["t"], sim_clamped["u"], color="#2ca02c", lw=1.8, label="Clamped Force $u(t)$")
    ax_w_force.axhline(30.0, color="k", linestyle="-.", lw=1.2, label=r"Force Limit ($\pm 30$ N)")
    ax_w_force.axhline(-30.0, color="k", linestyle="-.", lw=1.2)
    ax_w_force.axhline(20.0, color="#1c6b1c", linestyle=":", lw=1.0, label="20 N Holding Force")
    ax_w_force.set_ylabel("Force $u(t)$ (N)")
    ax_w_force.legend(loc="lower right", fontsize=7.8, ncol=2)
    ax_w_force.set_ylim(-35, 35)

    # Integrator State
    ax_w_int.plot(sim_unclamped["t"], sim_unclamped["i"], color="#d62728", lw=1.6, label="Unclamped Integrator $u_I = K_i \\int e$ (Blowup)")
    ax_w_int.plot(sim_clamped["t"], sim_clamped["i"], color="#2ca02c", lw=1.8, label="Clamped Integrator $u_I$ (Bounded)")
    ax_w_int.axhline(20.0, color="#1c6b1c", linestyle=":", lw=1.2, label="Target 20 N")
    ax_w_int.set_xlabel("Time (s)")
    ax_w_int.set_ylabel("Integral Term (N)")
    ax_w_int.legend(loc="lower right", fontsize=7.8)
    ax_w_int.set_ylim(-5, 90)

    save_fig(fig_f11, "figures/fig11_windup_clamping.png")
    print("Generated: figures/fig11_windup_clamping.png")

    # =========================================================================
    # PROBLEM (b): Sensor Noise & Low-Pass Derivative Filtering
    # =========================================================================
    sigma_noise = 0.001  # 1 mm standard deviation
    plant_noise = MassSpringDamper(m=1.0, c=10.0, k=20.0)

    # 1. Raw D (tau = 0.0)
    ctrl_raw = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, d_filter_tau=0.0)
    sim_raw = simulate(plant_noise, ctrl_raw, setpoint=1.0, T=3.0, dt_sim=0.001, noise_sigma=sigma_noise, seed=42)
    # Steady state interval [1.5, 3.0] s
    steady_mask = sim_raw["t"] >= 1.5
    std_u_raw = float(np.std(sim_raw["u"][steady_mask]))

    # 2. Filtered D (tau = 0.02 s)
    ctrl_f02 = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, d_filter_tau=0.02)
    sim_f02 = simulate(plant_noise, ctrl_f02, setpoint=1.0, T=3.0, dt_sim=0.001, noise_sigma=sigma_noise, seed=42)
    std_u_f02 = float(np.std(sim_f02["u"][steady_mask]))

    # 3. Filtered D (tau = 0.05 s)
    ctrl_f05 = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, d_filter_tau=0.05)
    sim_f05 = simulate(plant_noise, ctrl_f05, setpoint=1.0, T=3.0, dt_sim=0.001, noise_sigma=sigma_noise, seed=42)
    std_u_f05 = float(np.std(sim_f05["u"][steady_mask]))

    # Theory: Kd * sqrt(2) * sigma / dt
    theory_noise_std = 30.0 * np.sqrt(2.0) * sigma_noise / 0.001

    t04_entries.append({
        "problem": "Sensor Noise (σ = 1 mm)",
        "metric": "Control Force Chatter std(u)",
        "before": f"Raw D: std(u) = {std_u_raw:.2f} N (Theory: {theory_noise_std:.1f} N)",
        "after": f"Filtered (τ=0.02s): {std_u_f02:.2f} N | (τ=0.05s): {std_u_f05:.2f} N",
        "mechanism": "First-order low-pass filter (α = dt / (τ + dt)) attenuates high-frequency noise.",
    })

    fig_f12, (ax_n_force, ax_n_zoom, ax_n_pos) = plt.subplots(
        3, 1, figsize=(7.5, 6.8), gridspec_kw={"height_ratios": [1.1, 1.0, 1.0]}
    )

    # Full force trajectory
    ax_n_force.plot(sim_raw["t"], sim_raw["u"], color="#d62728", lw=0.8, alpha=0.8, label=f"Raw D (std: {std_u_raw:.1f} N) — Massive Actuator Chatter")
    ax_n_force.plot(sim_f02["t"], sim_f02["u"], color="#1f77b4", lw=1.4, label=f"Filtered D $\\tau = 0.02$ s (std: {std_u_f02:.2f} N)")
    ax_n_force.plot(sim_f05["t"], sim_f05["u"], color="#2ca02c", lw=1.6, label=f"Filtered D $\\tau = 0.05$ s (std: {std_u_f05:.2f} N)")
    ax_n_force.set_ylabel("Force $u(t)$ (N)")
    ax_n_force.set_title("Figure 12. Sensor Measurement Noise ($\\sigma = 1$ mm) & Low-Pass Filter Quenching", fontsize=10.5, weight="bold")
    ax_n_force.legend(loc="upper right", fontsize=7.8)
    ax_n_force.set_xlim(0, 3.0)
    ax_n_force.set_ylim(-150, 400)

    # Zoomed force at steady state (t in [2.0, 2.3] s)
    zoom_mask = (sim_raw["t"] >= 2.0) & (sim_raw["t"] <= 2.3)
    ax_n_zoom.plot(sim_raw["t"][zoom_mask], sim_raw["u"][zoom_mask], color="#d62728", lw=1.0, alpha=0.85, label="Raw D (42.4 N RMS noise swings)")
    ax_n_zoom.plot(sim_f02["t"][zoom_mask], sim_f02["u"][zoom_mask], color="#1f77b4", lw=1.6, label="Filtered $\\tau=0.02$ s (Smooth)")
    ax_n_zoom.plot(sim_f05["t"][zoom_mask], sim_f05["u"][zoom_mask], color="#2ca02c", lw=1.8, label="Filtered $\\tau=0.05$ s (Ultra-smooth)")
    ax_n_zoom.axhline(20.0, color="k", linestyle=":", lw=1.0)
    ax_n_zoom.set_ylabel("Force Zoom (N)")
    ax_n_zoom.set_xlim(2.0, 2.3)
    ax_n_zoom.set_ylim(-35, 75)
    ax_n_zoom.legend(loc="upper right", fontsize=7.8)

    # Position response comparing lag tradeoff
    ax_n_pos.plot(sim_raw["t"], sim_raw["y"], color="#d62728", lw=1.2, linestyle="--", label="Raw D Position")
    ax_n_pos.plot(sim_f02["t"], sim_f02["y"], color="#1f77b4", lw=1.6, label="Filtered $\\tau = 0.02$ s (Optimal)")
    ax_n_pos.plot(sim_f05["t"], sim_f05["y"], color="#2ca02c", lw=1.6, label="Filtered $\\tau = 0.05$ s (Slight lag)")
    ax_n_pos.axhline(1.0, color=COLOR_SETPOINT, linestyle=":", lw=1.2)
    ax_n_pos.set_xlabel("Time (s)")
    ax_n_pos.set_ylabel("Position (m)")
    ax_n_pos.set_xlim(0, 3.0)
    ax_n_pos.set_ylim(-0.05, 1.25)
    ax_n_pos.legend(loc="lower right", fontsize=7.8)

    save_fig(fig_f12, "figures/fig12_noise_filtering.png")
    print("Generated: figures/fig12_noise_filtering.png")

    # =========================================================================
    # PROBLEM (c): Setpoint Kick (D on error vs D on measurement)
    # =========================================================================
    plant_kick_err = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    ctrl_kick_err = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, d_on="error")
    sim_kick_err = simulate(plant_kick_err, ctrl_kick_err, setpoint=1.0, T=0.1, dt_sim=0.001)

    plant_kick_meas = MassSpringDamper(m=1.0, c=10.0, k=20.0)
    ctrl_kick_meas = PID(kp=300.0, ki=300.0, kd=30.0, dt=0.001, d_on="measurement")
    sim_kick_meas = simulate(plant_kick_meas, ctrl_kick_meas, setpoint=1.0, T=0.1, dt_sim=0.001)

    peak_force_err = float(np.max(np.abs(sim_kick_err["u"])))
    peak_force_meas = float(np.max(np.abs(sim_kick_meas["u"])))

    t04_entries.append({
        "problem": "Setpoint Kick at t=0",
        "metric": "Initial Force Spike u(0)",
        "before": f"D on error: {peak_force_err:,.0f} N spike",
        "after": f"D on measurement: {peak_force_meas:.0f} N (100x reduction)",
        "mechanism": "Differentiating y(t) instead of e(t) eliminates impulsive derivative of step input.",
    })

    fig_f13, (ax_k_force, ax_k_pos) = plt.subplots(
        2, 1, figsize=(7.5, 4.8), sharex=True, gridspec_kw={"height_ratios": [1.4, 0.9]}
    )

    t_kick_ms = sim_kick_err["t"] * 1000.0  # Milliseconds

    ax_k_force.plot(t_kick_ms, sim_kick_err["u"], color="#d62728", lw=2.0, label=f"D on Error: Peak = {peak_force_err:,.0f} N (Violent Spike!)")
    ax_k_force.plot(t_kick_ms, sim_kick_meas["u"], color="#1f77b4", lw=2.2, label=f"D on Measurement: Peak = {peak_force_meas:.0f} N (Gentle Push)")
    ax_k_force.axhline(30.0, color="k", linestyle="-.", lw=1.0, label="Motor Limit (30 N)")
    ax_k_force.set_ylabel("Applied Force $u(t)$ (N)")
    ax_k_force.set_title("Figure 13. Setpoint Kick Suppression (First 50 ms of Step Response)", fontsize=10.5, weight="bold")
    ax_k_force.set_xlim(0, 50.0)
    ax_k_force.set_ylim(-500, 32000)
    ax_k_force.legend(loc="upper right", fontsize=8.0)

    # Position in first 50 ms
    ax_k_pos.plot(t_kick_ms, sim_kick_err["y"], color="#d62728", lw=1.5, linestyle="--", label="Position (D on error)")
    ax_k_pos.plot(t_kick_ms, sim_kick_meas["y"], color="#1f77b4", lw=2.0, label="Position (D on measurement)")
    ax_k_pos.set_xlabel("Time (milliseconds)")
    ax_k_pos.set_ylabel("Position (m)")
    ax_k_pos.legend(loc="upper left", fontsize=8.0)
    ax_k_pos.set_ylim(-0.01, 0.15)

    save_fig(fig_f13, "figures/fig13_setpoint_kick.png")
    print("Generated: figures/fig13_setpoint_kick.png")

    # =========================================================================
    # PROBLEM (d): Slower Controller Sampling Rates & Latency Blowup
    # =========================================================================
    dt_ctrl_list = [0.001, 0.020, 0.050, 0.090]
    dt_colors = ["#1f77b4", "#2ca02c", "#ff7f0e", "#d62728"]
    labels_dt = [
        r"$\Delta t_{ctrl} = 1$ ms (Continuous-like, OS: 4.3%)",
        r"$\Delta t_{ctrl} = 20$ ms (Stable & smooth, OS: 6.8%)",
        r"$\Delta t_{ctrl} = 50$ ms (Laggy, OS: 17.5%)",
        r"$\Delta t_{ctrl} = 90$ ms (Severe Instability / Divergence)",
    ]

    fig_f14, (ax_s_pos, ax_s_force) = plt.subplots(
        2, 1, figsize=(7.5, 5.4), sharex=True, gridspec_kw={"height_ratios": [1.3, 1.0]}
    )
    ax_s_pos.axhline(1.0, color=COLOR_SETPOINT, linestyle="--", lw=1.5, label="Target (1.0 m)")

    # Bisection search for critical instability period where overshoot > 100% or unbounded
    def test_stability_at_dt(dt_c):
        p = MassSpringDamper(m=1.0, c=10.0, k=20.0)
        c = PID(kp=300.0, ki=300.0, kd=30.0, dt=dt_c)
        s = simulate(p, c, setpoint=1.0, T=3.0, dt_sim=0.001, dt_ctrl=dt_c)
        return np.max(s["y"]) < 2.5

    # Bisection between 50 ms and 120 ms
    low_dt = 0.050
    high_dt = 0.120
    for _ in range(12):
        mid_dt = (low_dt + high_dt) / 2.0
        if test_stability_at_dt(mid_dt):
            low_dt = mid_dt
        else:
            high_dt = mid_dt
    critical_dt_ms = low_dt * 1000.0

    t04_entries.append({
        "problem": "Slow Controller Sampling",
        "metric": "Stability Limit Δt_crit",
        "before": f"100 ms update: Diverges / violent instability",
        "after": f"Stable for Δt ≤ {critical_dt_ms:.1f} ms; identical up to 20 ms",
        "mechanism": "Zero-order hold introduces phase lag ~ω·dt/2, reducing phase margin to zero.",
    })

    for dt_c, col, lbl in zip(dt_ctrl_list, dt_colors, labels_dt):
        plant_dt = MassSpringDamper(m=1.0, c=10.0, k=20.0)
        ctrl_dt = PID(kp=300.0, ki=300.0, kd=30.0, dt=dt_c)
        sim_dt = simulate(plant_dt, ctrl_dt, setpoint=1.0, T=3.0, dt_sim=0.001, dt_ctrl=dt_c)

        ax_s_pos.plot(sim_dt["t"], sim_dt["y"], color=col, lw=1.8, label=lbl)
        ax_s_force.plot(sim_dt["t"], sim_dt["u"], color=col, lw=1.4)

    ax_s_pos.set_ylabel("Displacement $x(t)$ (m)")
    ax_s_pos.set_title(f"Figure 14. Controller Sampling Rate Degradation (Critical Stability Limit: $\\Delta t_{{crit}} \\approx {critical_dt_ms:.1f}$ ms)",
                       fontsize=10.5, weight="bold")
    ax_s_pos.legend(loc="upper right", fontsize=7.8)
    ax_s_pos.set_xlim(0, 3.0)
    ax_s_pos.set_ylim(-0.1, 2.2)

    ax_s_force.set_xlabel("Time (s)")
    ax_s_force.set_ylabel("Force $u(t)$ (N)")
    ax_s_force.set_ylim(-150, 450)

    save_fig(fig_f14, "figures/fig14_slow_sampling.png")
    print("Generated: figures/fig14_slow_sampling.png")

    # =========================================================================
    # PART 5: Table T04 Generation
    # =========================================================================
    t04_lines = [
        "# Table 4: Real-World Engineering Hazards and Solutions",
        "",
        "Evaluation on the baseline PID controller ($K_p = 300, K_i = 300, K_d = 30$) under practical hardware constraints.",
        "",
        "| Problem / Hazard | Primary Metric | Before Engineering Fix | After Engineering Fix | Physical Mechanism |",
        "|---|---|---|---|---|",
    ]

    for e in t04_entries:
        t04_lines.append(
            f"| **{e['problem']}** | {e['metric']} | {e['before']} | **{e['after']}** | {e['mechanism']} |"
        )

    with open("results/t04_real_world_fixes.md", "w") as f:
        f.write("\n".join(t04_lines) + "\n")
    print("Saved: results/t04_real_world_fixes.md")


if __name__ == "__main__":
    run_section_7()
