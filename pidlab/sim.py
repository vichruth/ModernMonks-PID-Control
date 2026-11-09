"""Simulation runner for closed-loop dynamic systems.

Features:
  - Multi-rate simulation: plant integrated at dt_sim, controller evaluated at dt_ctrl (Zero-Order Hold)
  - Gaussian sensor noise injection (reproducible with random seed)
  - Complete time-series history logging of setpoint, output, noise, force, and PID decompositions
"""

from typing import Any, Callable, Dict, Union
import numpy as np


def simulate(
    plant: Any,
    controller: Any,
    setpoint: Union[float, Callable[[float], float], np.ndarray],
    T: float = 15.0,
    dt_sim: float = 0.001,
    dt_ctrl: float = None,
    noise_sigma: float = 0.0,
    seed: int = 42,
) -> Dict[str, np.ndarray]:
    """Execute closed-loop simulation over time span [0, T].

    Parameters:
        plant: Dynamic model with .step(u, dt), .reset(), and .output
        controller: PID controller with .update(r, y), .reset()
        setpoint: Constant scalar, time-varying function r(t), or trajectory array
        T: Total simulation duration in seconds (default 15.0 s)
        dt_sim: Plant integration step size (default 1 ms = 0.001 s)
        dt_ctrl: Controller update period (default None -> dt_sim). If > dt_sim,
                 simulates slower digital controller via Zero-Order Hold.
        noise_sigma: Standard deviation of Gaussian sensor measurement noise
        seed: Random seed for noise reproducibility

    Returns:
        dict containing numpy arrays:
            't': time vector (s)
            'y': true plant output / displacement (m)
            'y_meas': noisy sensor measurement fed to controller (m)
            'u': control effort / force applied by actuator (N)
            'p': proportional contribution (N)
            'i': integral contribution (N)
            'd': derivative contribution (N)
            'r': setpoint trajectory (m)
    """
    if dt_ctrl is None:
        dt_ctrl = dt_sim

    if hasattr(controller, "dt") and controller.dt != dt_ctrl:
        controller.dt = dt_ctrl

    n_steps_total = int(np.round(T / dt_sim)) + 1
    t = np.linspace(0.0, T, n_steps_total)

    # Resolve setpoint array
    if callable(setpoint):
        r = np.array([float(setpoint(ti)) for ti in t], dtype=float)
    elif isinstance(setpoint, (int, float)):
        r = np.full(n_steps_total, float(setpoint), dtype=float)
    else:
        r = np.asarray(setpoint, dtype=float)
        if len(r) != n_steps_total:
            raise ValueError(f"Setpoint length ({len(r)}) does not match time steps ({n_steps_total})")

    # Generate Gaussian sensor noise
    rng = np.random.default_rng(seed)
    if noise_sigma > 0.0:
        noise = rng.normal(0.0, float(noise_sigma), size=n_steps_total)
    else:
        noise = np.zeros(n_steps_total, dtype=float)

    # Pre-allocate trajectory buffers
    y = np.zeros(n_steps_total, dtype=float)
    y_meas = np.zeros(n_steps_total, dtype=float)
    u = np.zeros(n_steps_total, dtype=float)
    p = np.zeros(n_steps_total, dtype=float)
    i = np.zeros(n_steps_total, dtype=float)
    d = np.zeros(n_steps_total, dtype=float)

    # Reset states
    plant.reset()
    controller.reset()

    # Zero-order hold interval
    ctrl_step_interval = max(1, int(np.round(dt_ctrl / dt_sim)))

    u_hold = 0.0
    p_hold = 0.0
    i_hold = 0.0
    d_hold = 0.0

    for k in range(n_steps_total):
        # 1. Physical plant output
        y_true = plant.output
        y[k] = y_true

        # 2. Sensor measurement with noise
        measured = y_true + noise[k]
        y_meas[k] = measured

        # 3. Controller execution (Zero-Order Hold decimation)
        if k % ctrl_step_interval == 0:
            u_hold = controller.update(r[k], measured)
            p_hold = getattr(controller, "last_p", 0.0)
            i_hold = getattr(controller, "last_i", 0.0)
            d_hold = getattr(controller, "last_d", 0.0)

        # 4. Record applied values
        u[k] = u_hold
        p[k] = p_hold
        i[k] = i_hold
        d[k] = d_hold

        # 5. Advance physics to next timestep
        if k < n_steps_total - 1:
            plant.step(u_hold, dt_sim)

    return {
        "t": t,
        "y": y,
        "y_meas": y_meas,
        "u": u,
        "p": p,
        "i": i,
        "d": d,
        "r": r,
    }

