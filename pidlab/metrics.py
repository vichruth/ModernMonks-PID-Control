"""Performance metrics calculation for step responses.

Definitions strictly follow project specifications:
  - Rise time: time to go from 10% to 90% of setpoint (or None if never reached)
  - Overshoot: (peak - setpoint) / setpoint * 100% (or 0.0% if peak <= setpoint)
  - Settling time: last time |y - setpoint| > 2% of setpoint (or None if never settles)
  - Steady-state error: setpoint - mean(y over last 5% of samples)
  - Peak force: max(|u|) over the simulation run
"""

from typing import Dict, List, Optional, Union
import numpy as np


def find_peaks_simple(y: np.ndarray, threshold: Optional[float] = None) -> List[int]:
    """Lightweight 5-line local maxima finder (pure NumPy, no scipy)."""
    peaks = []
    for i in range(1, len(y) - 1):
        if y[i] > y[i - 1] and y[i] >= y[i + 1]:
            if threshold is None or y[i] > threshold:
                peaks.append(i)
    return peaks


def calc_metrics(
    t: np.ndarray,
    y: np.ndarray,
    r: Union[float, np.ndarray],
    u: Optional[np.ndarray] = None,
    settling_band: float = 0.02,
    final_window_ratio: float = 0.05,
) -> Dict[str, Union[float, Optional[float]]]:
    """Compute step response performance metrics against target setpoint.

    Parameters:
        t: Time vector (s)
        y: Plant output trajectory (m)
        r: Target setpoint scalar or array (m)
        u: Applied actuator force array (N), optional
        settling_band: Relative error band for settling (default 0.02 = 2%)
        final_window_ratio: Fraction of tail samples for steady-state (default 0.05 = 5%)

    Returns:
        dict with keys:
            'rise_time': float (s) or None if 90% setpoint is never reached
            'overshoot': float (%) relative to target setpoint (0.0 if no overshoot)
            'settling_time': float (s) or None if output never stays within settling band
            'steady_state_error': float (m)
            'peak_force': float (N) or None if u not supplied
            'peak_y': float (m)
    """
    final_target = float(r[-1]) if isinstance(r, np.ndarray) else float(r)

    # 1. Steady-state error (mean of last 5% of samples)
    tail_count = max(1, int(len(y) * final_window_ratio))
    y_ss = float(np.mean(y[-tail_count:]))
    steady_state_error = final_target - y_ss

    # 2. Overshoot relative to setpoint
    peak_y = float(np.max(y))
    if final_target > 0.0 and peak_y > final_target:
        overshoot = ((peak_y - final_target) / final_target) * 100.0
    else:
        overshoot = 0.0

    # 3. Rise time (10% to 90% of final setpoint)
    y_10 = 0.10 * final_target
    y_90 = 0.90 * final_target

    idx_10 = np.where(y >= y_10)[0]
    idx_90 = np.where(y >= y_90)[0]

    if len(idx_10) > 0 and len(idx_90) > 0:
        rise_time = float(t[idx_90[0]] - t[idx_10[0]])
    else:
        rise_time = None  # Never reaches 90%

    # 4. Settling time (last time |y - target| > settling_band * target)
    band_limit = abs(final_target * settling_band)
    error_abs = np.abs(y - final_target)
    outside_band_indices = np.where(error_abs > band_limit)[0]

    if len(outside_band_indices) == 0:
        settling_time = 0.0
    else:
        last_outside_idx = outside_band_indices[-1]
        if last_outside_idx == len(y) - 1:
            settling_time = None  # Never settles within 2% band
        else:
            settling_time = float(t[last_outside_idx + 1])

    # 5. Peak force
    peak_force = float(np.max(np.abs(u))) if u is not None else None

    return {
        "rise_time": rise_time,
        "overshoot": overshoot,
        "settling_time": settling_time,
        "steady_state_error": steady_state_error,
        "peak_force": peak_force,
        "peak_y": peak_y,
    }


def format_metrics(metrics: Dict[str, Union[float, Optional[float]]]) -> Dict[str, str]:
    """Format numerical metrics into publication-ready readable strings."""
    rt = metrics["rise_time"]
    st = metrics["settling_time"]
    pf = metrics["peak_force"]

    return {
        "rise_time": f"{rt:.3f} s" if rt is not None else "n/a (never reaches 90%)",
        "overshoot": f"{metrics['overshoot']:.2f}%",
        "settling_time": f"{st:.3f} s" if st is not None else "never settles",
        "steady_state_error": f"{metrics['steady_state_error']:.4f} m",
        "peak_force": f"{pf:.2f} N" if pf is not None else "n/a",
    }

