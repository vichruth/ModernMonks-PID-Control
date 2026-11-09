"""Interactive CLI Playground & Interview Practice Tool.

Allows rapid testing of parameter what-ifs from the command line:
    python3 playground.py --kp 300 --ki 300 --kd 30 --mass 2 --noise 0.001 --umax 30 --no-antiwindup

Prints step response performance metrics and optionally generates a diagnostic plot.
"""

import argparse
import sys
import numpy as np

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
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(description="PID Control Interactive Playground")
    # Controller gains
    parser.add_argument("--kp", type=float, default=300.0, help="Proportional gain Kp [N/m]")
    parser.add_argument("--ki", type=float, default=300.0, help="Integral gain Ki [N/(m*s)]")
    parser.add_argument("--kd", type=float, default=30.0, help="Derivative gain Kd [N*s/m]")
    # Plant parameters
    parser.add_argument("--mass", type=float, default=1.0, help="Block mass m [kg]")
    parser.add_argument("--c", type=float, default=10.0, help="Damping coefficient c [N*s/m]")
    parser.add_argument("--k", type=float, default=20.0, help="Spring stiffness k [N/m]")
    # Simulation options
    parser.add_argument("--target", type=float, default=1.0, help="Target setpoint displacement [m]")
    parser.add_argument("--tmax", type=float, default=15.0, help="Simulation duration [s] (default: 15.0 s)")
    parser.add_argument("--dt", type=float, default=0.001, help="Simulation integration timestep [s]")
    parser.add_argument("--dtctrl", type=float, default=None, help="Controller update interval [s]")
    parser.add_argument("--noise", type=float, default=0.0, help="Sensor noise std deviation sigma [m]")
    parser.add_argument("--umax", type=float, default=np.inf, help="Actuator upper saturation limit [N]")
    parser.add_argument("--umin", type=float, default=None, help="Actuator lower saturation limit [N] (defaults to -umax)")
    parser.add_argument("--no-antiwindup", action="store_true", help="Disable anti-windup clamping")
    parser.add_argument("--d-on", choices=["measurement", "error"], default="measurement", help="Derivative input signal")
    parser.add_argument("--d-tau", "--kd-tau", dest="d_tau", type=float, default=0.0, help="Derivative low-pass filter time constant [s]")
    parser.add_argument("--save", type=str, default="figures/playground.png", help="Save figure to file (default: 'figures/playground.png')")
    parser.add_argument("--no-save", action="store_true", help="Disable saving plot figure")
    parser.add_argument("--plot", action="store_true", help="Display static plot window")
    parser.add_argument("--interactive", "--gui", dest="interactive", action="store_true", help="Launch live interactive Matplotlib Slider GUI")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.interactive:
        from interactive_gui import launch_gui
        umax_init = 30.0 if not np.isfinite(args.umax) else args.umax
        fig, _ = launch_gui(
            kp0=args.kp,
            ki0=args.ki,
            kd0=args.kd,
            m0=args.mass,
            umax0=umax_init,
            noise0=args.noise,
        )
        plt.suptitle("Modern Monk Labs PID Interactive Visualizer", fontsize=12, fontweight="bold", y=0.98)
        plt.show()
        return

    # Configure plant & controller
    plant = MassSpringDamper(m=args.mass, c=args.c, k=args.k)
    anti_windup = "none" if args.no_antiwindup else "clamp"
    u_min = -args.umax if args.umin is None and np.isfinite(args.umax) else (args.umin if args.umin is not None else -np.inf)

    controller = PID(
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        dt=args.dt if args.dtctrl is None else args.dtctrl,
        u_min=u_min,
        u_max=args.umax,
        anti_windup=anti_windup,
        d_on=args.d_on,
        d_filter_tau=args.d_tau,
    )

    # Run simulation
    sim = simulate(
        plant=plant,
        controller=controller,
        setpoint=args.target,
        T=args.tmax,
        dt_sim=args.dt,
        dt_ctrl=args.dtctrl,
        noise_sigma=args.noise,
    )

    # Compute metrics
    m = calc_metrics(sim["t"], sim["y"], sim["r"], u=sim["u"])
    fm = format_metrics(m)

    # Print summary table
    print("\n" + "=" * 50)
    print("       PID LAB PLAYGROUND SIMULATION RESULT       ")
    print("=" * 50)
    print(f"  Configuration:")
    print(f"    Gains:       Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    print(f"    Plant:       m={args.mass} kg, c={args.c} N*s/m, k={args.k} N/m")
    print(f"    Actuator:    Limits=[{u_min}, {args.umax}] N, Anti-Windup={anti_windup}")
    print(f"    Sensor:      Noise sigma={args.noise} m, D-Input={args.d_on}, Filter tau={args.d_tau} s")
    print("-" * 50)
    print(f"  Performance Metrics:")
    print(f"    Rise Time (10%->90%):   {fm['rise_time']}")
    print(f"    Overshoot:              {fm['overshoot']}")
    print(f"    Settling Time (2%):     {fm['settling_time']}")
    print(f"    Steady-State Error:     {fm['steady_state_error']}")
    print(f"    Peak Force:             {fm['peak_force']}")
    print("=" * 50 + "\n")

    save_path = None if args.no_save else args.save

    if save_path or args.plot:
        setup_style()
        fig, (ax_pos, ax_force) = plt.subplots(2, 1, figsize=(7.5, 5.5), sharex=True)

        # Position plot
        ax_pos.plot(sim["t"], sim["r"], color=COLOR_SETPOINT, linestyle="--", label="Target (1 m)")
        if args.noise > 0.0:
            ax_pos.plot(sim["t"], sim["y_meas"], color="#aaaaaa", alpha=0.5, label="Measured (noisy)")
        ax_pos.plot(sim["t"], sim["y"], color=COLOR_OUTPUT, label=f"Position x(t) [OS: {fm['overshoot']}, ts: {fm['settling_time']}]")
        ax_pos.set_ylabel("Displacement (m)")
        ax_pos.set_title(f"Response: Kp={args.kp}, Ki={args.ki}, Kd={args.kd} (m={args.mass}kg)")
        ax_pos.legend(loc="lower right", fontsize=8.0)

        # Force decomposition plot
        ax_force.plot(sim["t"], sim["p"], color=COLOR_P, label="P contribution")
        ax_force.plot(sim["t"], sim["i"], color=COLOR_I, label="I contribution")
        ax_force.plot(sim["t"], sim["d"], color=COLOR_D, label="D contribution")
        ax_force.plot(sim["t"], sim["u"], color=COLOR_FORCE, linestyle=":", label=f"Total u (Peak: {fm['peak_force']})")
        if np.isfinite(args.umax):
            ax_force.axhline(args.umax, color="k", linestyle="-.", alpha=0.5, label=f"Limit ({args.umax} N)")
            if np.isfinite(u_min):
                ax_force.axhline(u_min, color="k", linestyle="-.", alpha=0.5)
        ax_force.set_xlabel("Time (s)")
        ax_force.set_ylabel("Force (N)")
        ax_force.legend(loc="upper right", fontsize=7.8)

        if save_path:
            import os
            os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
            save_fig(fig, save_path)
            print(f"Plot saved to: {save_path}")
        if args.plot:
            plt.show()


if __name__ == "__main__":
    main()
