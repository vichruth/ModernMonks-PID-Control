"""pidlab: Core control simulation and analysis toolkit for PID studies."""

from pidlab.plants import MassSpringDamper, ThreeLags
from pidlab.pid import PID
from pidlab.sim import simulate
from pidlab.metrics import calc_metrics, format_metrics, find_peaks_simple
from pidlab.style import (
    setup_style,
    save_fig,
    COLOR_P,
    COLOR_I,
    COLOR_D,
    COLOR_SETPOINT,
    COLOR_OUTPUT,
    COLOR_FORCE,
    COLOR_BAND,
    COLOR_ACCENT,
)

__all__ = [
    "MassSpringDamper",
    "ThreeLags",
    "PID",
    "simulate",
    "calc_metrics",
    "format_metrics",
    "find_peaks_simple",
    "setup_style",
    "save_fig",
    "COLOR_P",
    "COLOR_I",
    "COLOR_D",
    "COLOR_SETPOINT",
    "COLOR_OUTPUT",
    "COLOR_FORCE",
    "COLOR_BAND",
    "COLOR_ACCENT",
]
