"""Visual styling and plotting utilities for publication-grade figures.

Follows strict project style rules:
  - P = blue (#1f77b4)
  - I = green (#2ca02c)
  - D = red (#d62728)
  - Setpoint = black dashed (#000000, '--')
  - Total output / response = dark grey (#333333)
  - Applied force u = orange (#ff7f0e)
  - Figures saved as 300 DPI PNGs formatted for report inclusion (6.5 in width)
"""

import os
import matplotlib
# Headless mode for reproducible server / headless execution
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Consistent project palette
COLOR_P = "#1f77b4"          # Blue
COLOR_I = "#2ca02c"          # Green
COLOR_D = "#d62728"          # Red
COLOR_SETPOINT = "#000000"   # Black dashed
COLOR_OUTPUT = "#333333"     # Dark grey
COLOR_FORCE = "#ff7f0e"      # Orange
COLOR_BAND = "#e0e0e0"       # Light grey for settling bands
COLOR_ACCENT = "#9467bd"     # Purple for comparisons


def setup_style() -> None:
    """Apply consistent font, grid, and aesthetic settings across all plots."""
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.size": 9.5,
        "axes.titlesize": 10.5,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 8.5,
        "figure.titlesize": 11.5,
        "figure.titleweight": "bold",
        "figure.dpi": 300,
        "lines.linewidth": 1.6,
        "axes.grid": True,
        "grid.alpha": 0.35,
        "grid.linestyle": ":",
        "axes.axisbelow": True,
    })


def save_fig(fig: plt.Figure, filepath: str) -> None:
    """Save figure at 300 DPI with tight bounding box and close it."""
    os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
    try:
        fig.tight_layout()
    except Exception:
        pass
    fig.savefig(filepath, dpi=300, bbox_inches="tight")
    plt.close(fig)
