"""Figure F01: Control loop block diagram.

Illustrates the negative feedback loop architecture:
  Target (r) -> Error (e) -> Controller (PID) -> Actuator Force (u) -> Machine (Plant) -> Output (y) -> Sensor -> Feedback (-)
Also provides the physical analogy mapping (e.g. Shower temperature adjustment).
"""

import os
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from pidlab.style import setup_style, save_fig, COLOR_P, COLOR_OUTPUT, COLOR_SETPOINT


def generate_loop_diagram(output_path: str = "figures/fig01_loop_diagram.png"):
    setup_style()
    fig, ax = plt.subplots(figsize=(8.0, 3.8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5)
    ax.axis("off")

    # Box styles
    box_kw = dict(boxstyle="round,pad=0.25,rounding_size=0.15", ec="#2b2b2b", lw=1.8, fc="#f4f6f8")
    ctrl_kw = dict(boxstyle="round,pad=0.25,rounding_size=0.15", ec="#1f77b4", lw=2.0, fc="#e8f1f8")
    plant_kw = dict(boxstyle="round,pad=0.25,rounding_size=0.15", ec="#2ca02c", lw=2.0, fc="#eaf5ea")
    sensor_kw = dict(boxstyle="round,pad=0.25,rounding_size=0.15", ec="#7f7f7f", lw=1.8, fc="#f0f0f0")

    # 1. Summing junction (circle at x=2.0, y=3.2)
    sum_center = (2.0, 3.2)
    sum_circle = Circle(sum_center, 0.28, ec="#2b2b2b", fc="#ffffff", lw=1.8)
    ax.add_patch(sum_circle)
    ax.text(sum_center[0], sum_center[1], r"$\mathbf{\Sigma}$", ha="center", va="center", fontsize=13, weight="bold")
    ax.text(sum_center[0] - 0.42, sum_center[1] + 0.12, "+", fontsize=12, weight="bold")
    ax.text(sum_center[0] + 0.12, sum_center[1] - 0.45, "−", fontsize=14, weight="bold")

    # 2. Controller block (x=3.4 to 4.9, y=2.6 to 3.8)
    ctrl_box = FancyBboxPatch((3.4, 2.6), 1.5, 1.2, **ctrl_kw)
    ax.add_patch(ctrl_box)
    ax.text(4.15, 3.35, "Controller\n(PID)", ha="center", va="center", fontsize=10.5, weight="bold", color="#104e8b")
    ax.text(4.15, 2.85, r"$u(t) = K_p e + K_i \int e + K_d \dot{e}$", ha="center", va="center", fontsize=8.5)

    # 3. Plant block (x=6.0 to 7.8, y=2.6 to 3.8)
    plant_box = FancyBboxPatch((6.0, 2.6), 1.8, 1.2, **plant_kw)
    ax.add_patch(plant_box)
    ax.text(6.9, 3.35, "Machine / Plant\n(Spring-Mass-Damper)", ha="center", va="center", fontsize=10.5, weight="bold", color="#1c6b1c")
    ax.text(6.9, 2.85, r"$m \ddot{x} + c \dot{x} + k x = u$", ha="center", va="center", fontsize=8.5)

    # 4. Sensor block (x=4.7 to 6.3, y=0.8 to 1.8)
    sensor_box = FancyBboxPatch((4.7, 0.8), 1.6, 1.0, **sensor_kw)
    ax.add_patch(sensor_box)
    ax.text(5.5, 1.40, "Sensor", ha="center", va="center", fontsize=10, weight="bold", color="#333333")
    ax.text(5.5, 1.05, "Encoder / Position", ha="center", va="center", fontsize=8)

    # Arrows definition
    arrow_kw = dict(arrowstyle="->", lw=1.8, color="#2b2b2b", mutation_scale=15)

    # Input target r(t) -> Summing junction
    ax.annotate("", xy=(sum_center[0] - 0.28, sum_center[1]), xytext=(0.6, sum_center[1]), arrowprops=arrow_kw)
    ax.text(1.2, sum_center[1] + 0.22, "Target $r(t)$\n(Setpoint: 1 m)", ha="center", va="bottom", fontsize=9, weight="bold")

    # Summing junction -> Controller (Error e(t))
    ax.annotate("", xy=(3.4, sum_center[1]), xytext=(sum_center[0] + 0.28, sum_center[1]), arrowprops=arrow_kw)
    ax.text(2.82, sum_center[1] + 0.22, "Error $e(t)$\n$r(t) - y(t)$", ha="center", va="bottom", fontsize=8.5)

    # Controller -> Plant (Control force u(t))
    ax.annotate("", xy=(6.0, sum_center[1]), xytext=(4.9, sum_center[1]), arrowprops=arrow_kw)
    ax.text(5.45, sum_center[1] + 0.22, "Force $u(t)$\n(Newtons)", ha="center", va="bottom", fontsize=8.5, color="#ff7f0e", weight="bold")

    # Plant -> Output y(t)
    ax.annotate("", xy=(9.4, sum_center[1]), xytext=(7.8, sum_center[1]), arrowprops=arrow_kw)
    ax.text(8.8, sum_center[1] + 0.22, "Output $y(t)$\n(Position $x$)", ha="center", va="bottom", fontsize=9, weight="bold")

    # Feedback branch: from y branch (x=8.4) down to sensor (x=6.3)
    # y=3.2 down to y=1.3, then left to sensor
    ax.plot([8.4, 8.4], [3.2, 1.3], color="#2b2b2b", lw=1.8)
    ax.plot([8.4, 6.3], [1.3, 1.3], color="#2b2b2b", lw=1.8)
    ax.annotate("", xy=(6.3, 1.3), xytext=(6.35, 1.3), arrowprops=arrow_kw)

    # Sensor (x=4.7) to summing junction
    ax.plot([4.7, 2.0], [1.3, 1.3], color="#2b2b2b", lw=1.8)
    ax.annotate("", xy=(sum_center[0], sum_center[1] - 0.28), xytext=(sum_center[0], 1.3), arrowprops=arrow_kw)
    ax.text(3.35, 1.05, "Measured Position $y(t)$", ha="center", va="top", fontsize=8.5)

    # Real-world analogy box below
    analogy_text = (
        "Daily-Life Analogy (Taking a Shower):\n"
        "• Target r(t): Desired comfortable water temperature\n"
        "• Sensor: Skin senses actual temperature y(t)\n"
        "• Controller: Brain computes error (Too Cold / Hot) and decides adjustment\n"
        "• Actuator & Plant: Hand turns faucet valve u(t) to regulate hot/cold water flow"
    )
    ax.text(0.6, 0.45, analogy_text, fontsize=8, style="italic", bbox=dict(boxstyle="square,pad=0.4", fc="#fafafa", ec="#cccccc", lw=0.8))

    ax.set_title("Figure 1. Closed-Loop Negative Feedback Control Architecture", fontsize=11, weight="bold", pad=12)
    save_fig(fig, output_path)
    print(f"Generated: {output_path}")


if __name__ == "__main__":
    generate_loop_diagram()
