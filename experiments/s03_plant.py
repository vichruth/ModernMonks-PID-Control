"""Figure F03: Open-Loop Plant Physics and Equilibrium Holding Force.

Demonstrates:
  - Open-loop step responses to constant input forces: u = 0 N, 10 N, and 20 N.
  - At rest (v = 0, a = 0), damping does nothing (c*v = 0) and inertia does nothing (m*a = 0).
  - Only the spring resists motion: F_spring = k * x.
  - Therefore, holding x = 1.0 m requires exactly u = 20 N.
  - Shows that the open-loop plant is inherently overdamped (zeta = 1.118, poles at -2.76 and -7.24),
    meaning any oscillation seen later in closed loop is purely induced by the feedback controller.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from pidlab.plants import MassSpringDamper
from pidlab.style import setup_style, save_fig, COLOR_OUTPUT, COLOR_FORCE, COLOR_SETPOINT


def generate_plant_figure(output_path: str = "figures/fig03_open_loop_plant.png"):
    setup_style()

    dt = 0.001
    t = np.arange(0, 2.5 + dt / 2, dt)
    forces = [0.0, 10.0, 20.0]
    colors = ["#7f7f7f", "#1f77b4", "#2ca02c"]
    labels = ["$u = 0$ N $\\rightarrow x_{ss} = 0.0$ m",
              "$u = 10$ N $\\rightarrow x_{ss} = 0.5$ m",
              "$u = 20$ N $\\rightarrow x_{ss} = 1.0$ m"]

    fig, (ax_pos, ax_force) = plt.subplots(2, 1, figsize=(7.2, 5.2), sharex=True, gridspec_kw={"height_ratios": [1.4, 0.9]})

    for f_val, col, lab in zip(forces, colors, labels):
        plant = MassSpringDamper(m=1.0, c=10.0, k=20.0)
        x_history = []
        for _ in t:
            x_history.append(plant.step(u=f_val, dt=dt))

        ax_pos.plot(t, x_history, color=col, lw=2.0, label=lab)
        ax_pos.axhline(f_val / 20.0, color=col, linestyle=":", alpha=0.5)

        # Plot applied force step
        ax_force.plot(t, np.full_like(t, f_val), color=col, lw=1.8)

    # Position annotations
    ax_pos.set_ylabel("Displacement $x$ (m)")
    ax_pos.set_title("Open-Loop Step Responses: $m\\ddot{x} + c\\dot{x} + kx = u$ ($m=1$ kg, $c=10$ Ns/m, $k=20$ N/m)",
                     fontsize=10.5, weight="bold")
    ax_pos.legend(loc="center right", fontsize=8.5)
    ax_pos.set_xlim(0, 2.5)
    ax_pos.set_ylim(-0.05, 1.15)

    # Physics callout box
    physics_note = (
        "System Damping & Poles:\n"
        r"• $\omega_n = \sqrt{k/m} = \sqrt{20} \approx 4.47$ rad/s" "\n"
        r"• $\zeta = c / (2\sqrt{mk}) = 10 / (2\sqrt{20}) \approx 1.118$" "\n"
        r"• System is Overdamped ($\zeta > 1$): No bouncing on its own!" "\n"
        r"• Poles: $s = -2.76$ and $-7.24$" "\n"
        r"• Equilibrium: At rest, $\dot{x}=\ddot{x}=0 \Rightarrow u = k \cdot x_{ss}$"
    )
    ax_pos.text(0.04, 0.40, physics_note, transform=ax_pos.transAxes, fontsize=8.0,
                bbox=dict(boxstyle="round,pad=0.3", fc="#fdfdfd", ec="#bbbbbb", lw=0.9))

    # Force annotations
    ax_force.set_xlabel("Time (s)")
    ax_force.set_ylabel("Constant Force $u$ (N)")
    ax_force.set_ylim(-2, 25)
    ax_force.set_yticks([0, 10, 20])
    ax_force.text(1.3, 21.0, "20 N required to hold 1.0 m displacement", color="#1c6b1c", fontsize=8.5, weight="bold")

    save_fig(fig, output_path)
    print(f"Generated: {output_path}")


if __name__ == "__main__":
    generate_plant_figure()
