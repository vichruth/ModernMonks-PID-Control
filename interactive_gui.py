"""Interactive PID Playground GUI with Matplotlib Sliders.

Pure NumPy and Matplotlib only (no external UI libraries, 100% compliant with rules).
Provides real-time slider controls for Kp, Ki, Kd, mass, noise, and actuator limit
with instant visual feedback on step response, force decomposition, and performance metrics.

Usage:
    python3 interactive_gui.py
    python3 playground.py --interactive
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

from pidlab.plants import MassSpringDamper
from pidlab.pid import PID
from pidlab.sim import simulate
from pidlab.metrics import calc_metrics, format_metrics
from pidlab.style import (
    setup_style,
    COLOR_P,
    COLOR_I,
    COLOR_D,
    COLOR_SETPOINT,
    COLOR_OUTPUT,
    COLOR_FORCE,
)


def launch_gui(kp0=300.0, ki0=300.0, kd0=30.0, m0=1.0, umax0=30.0, noise0=0.0):
    setup_style()
    fig, (ax_pos, ax_force) = plt.subplots(2, 1, figsize=(9.0, 7.5), sharex=True)
    plt.subplots_adjust(left=0.12, right=0.92, top=0.90, bottom=0.38, hspace=0.25)

    t_eval = np.linspace(0, 15.0, 15001)

    # Initial simulation
    plant = MassSpringDamper(m=m0, c=10.0, k=20.0)
    controller = PID(
        kp=kp0,
        ki=ki0,
        kd=kd0,
        dt=0.001,
        u_min=-umax0,
        u_max=umax0,
        anti_windup="clamp",
        d_on="measurement",
        d_filter_tau=0.02,
    )
    res = simulate(plant, controller, setpoint=1.0, T=15.0, dt_sim=0.001, noise_sigma=noise0)
    met = calc_metrics(res["t"], res["y"], res["r"], u=res["u"])
    fm = format_metrics(met)

    # Plots
    line_target = ax_pos.axhline(1.0, color=COLOR_SETPOINT, linestyle="--", label="Target (1.0 m)")
    (line_pos,) = ax_pos.plot(res["t"], res["y"], color=COLOR_OUTPUT, lw=1.8, label="Position x(t)")
    ax_pos.set_ylabel("Position (m)")
    ax_pos.set_xlim(0, 15.0)
    ax_pos.set_ylim(-0.2, 1.8)
    ax_pos.grid(True, linestyle=":", alpha=0.6)
    title_text = ax_pos.set_title(
        f"Step Response | tr: {fm['rise_time']} | OS: {fm['overshoot']} | ts: {fm['settling_time']} | ess: {fm['steady_state_error']}",
        fontsize=10.0,
        fontweight="bold",
    )
    ax_pos.legend(loc="lower right", fontsize=8.5)

    (line_p,) = ax_force.plot(res["t"], res["p"], color=COLOR_P, lw=1.0, alpha=0.7, label="P term")
    (line_i,) = ax_force.plot(res["t"], res["i"], color=COLOR_I, lw=1.0, alpha=0.7, label="I term")
    (line_d,) = ax_force.plot(res["t"], res["d"], color=COLOR_D, lw=1.0, alpha=0.7, label="D term")
    (line_u,) = ax_force.plot(res["t"], res["u"], color=COLOR_FORCE, lw=1.5, label=f"Total u (Peak: {fm['peak_force']})")
    line_lim_pos = ax_force.axhline(umax0, color="k", linestyle="-.", alpha=0.5, label=f"+/-{umax0:.0f} N Limit")
    line_lim_neg = ax_force.axhline(-umax0, color="k", linestyle="-.", alpha=0.5)
    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel("Force (N)")
    ax_force.set_ylim(-60, 60)
    ax_force.grid(True, linestyle=":", alpha=0.6)
    leg_force = ax_force.legend(loc="upper right", fontsize=7.5)

    # Sliders layout
    ax_kp = plt.axes([0.18, 0.28, 0.65, 0.025])
    ax_ki = plt.axes([0.18, 0.23, 0.65, 0.025])
    ax_kd = plt.axes([0.18, 0.18, 0.65, 0.025])
    ax_m = plt.axes([0.18, 0.13, 0.65, 0.025])
    ax_umax = plt.axes([0.18, 0.08, 0.65, 0.025])
    ax_noise = plt.axes([0.18, 0.03, 0.65, 0.025])

    s_kp = Slider(ax_kp, "Kp (N/m)", 0.0, 1000.0, valinit=kp0, valstep=5.0)
    s_ki = Slider(ax_ki, "Ki (N/m*s)", 0.0, 600.0, valinit=ki0, valstep=5.0)
    s_kd = Slider(ax_kd, "Kd (N*s/m)", 0.0, 80.0, valinit=kd0, valstep=1.0)
    s_m = Slider(ax_m, "Mass (kg)", 0.2, 5.0, valinit=m0, valstep=0.1)
    s_umax = Slider(ax_umax, "Limit (N)", 10.0, 100.0, valinit=umax0, valstep=5.0)
    s_noise = Slider(ax_noise, "Noise (mm)", 0.0, 5.0, valinit=noise0 * 1000, valstep=0.1)

    ax_reset = plt.axes([0.86, 0.28, 0.08, 0.04])
    btn_reset = Button(ax_reset, "Reset", hovercolor="0.975")

    def update(val):
        kp = s_kp.val
        ki = s_ki.val
        kd = s_kd.val
        mass = s_m.val
        umax = s_umax.val
        noise = s_noise.val / 1000.0

        p = MassSpringDamper(m=mass, c=10.0, k=20.0)
        c = PID(
            kp=kp,
            ki=ki,
            kd=kd,
            dt=0.001,
            u_min=-umax,
            u_max=umax,
            anti_windup="clamp",
            d_on="measurement",
            d_filter_tau=0.02,
        )
        r = simulate(p, c, setpoint=1.0, T=15.0, dt_sim=0.001, noise_sigma=noise)
        m_eval = calc_metrics(r["t"], r["y"], r["r"], u=r["u"])
        fm_eval = format_metrics(m_eval)

        line_pos.set_ydata(r["y"])
        line_p.set_ydata(r["p"])
        line_i.set_ydata(r["i"])
        line_d.set_ydata(r["d"])
        line_u.set_ydata(r["u"])
        line_u.set_label(f"Total u (Peak: {fm_eval['peak_force']})")
        line_lim_pos.set_ydata([umax, umax])
        line_lim_neg.set_ydata([-umax, -umax])

        title_text.set_text(
            f"Step Response | tr: {fm_eval['rise_time']} | OS: {fm_eval['overshoot']} | ts: {fm_eval['settling_time']} | ess: {fm_eval['steady_state_error']}"
        )
        leg_force.get_texts()[-1].set_text(f"Total u (Peak: {fm_eval['peak_force']})")

        fig.canvas.draw_idle()

    s_kp.on_changed(update)
    s_ki.on_changed(update)
    s_kd.on_changed(update)
    s_m.on_changed(update)
    s_umax.on_changed(update)
    s_noise.on_changed(update)

    def reset(event):
        s_kp.reset()
        s_ki.reset()
        s_kd.reset()
        s_m.reset()
        s_umax.reset()
        s_noise.reset()

    btn_reset.on_clicked(reset)

    return fig, (s_kp, s_ki, s_kd, s_m, s_umax, s_noise, btn_reset)


def main():
    fig, widgets = launch_gui()
    plt.suptitle("Modern Monk Labs PID Interactive Visualizer", fontsize=12, fontweight="bold", y=0.98)
    plt.show()


if __name__ == "__main__":
    main()
