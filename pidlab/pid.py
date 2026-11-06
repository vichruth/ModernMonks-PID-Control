"""Parallel-form PID Controller.

Features:
  - Parallel formulation: u = Kp*e + Ki*integral(e) + Kd*de/dt
  - Switchable anti-windup: "clamp" (conditional integration), "none", "backcalc"
  - Switchable derivative input: "measurement" (eliminates setpoint kick) or "error"
  - Switchable first-order low-pass filter on derivative term (d_filter_tau)
  - Configurable actuator saturation bounds [u_min, u_max]
"""

from typing import Optional
import numpy as np


class PID:
    """Discrete Parallel PID Controller."""

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        dt: float = 0.001,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        anti_windup: str = "clamp",       # "none" | "clamp" | "backcalc"
        d_on: str = "measurement",        # "measurement" | "error"
        d_filter_tau: float = 0.0,        # seconds; 0.0 = unfiltered
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.dt = float(dt)
        self.u_min = float(u_min)
        self.u_max = float(u_max)
        self.anti_windup = str(anti_windup).lower()
        self.d_on = str(d_on).lower()
        self.d_filter_tau = float(d_filter_tau)

        self.reset()

    def reset(self) -> None:
        """Reset internal accumulator and state memories."""
        self.integral = 0.0
        self.prev_meas: Optional[float] = None
        self.prev_error: Optional[float] = None
        self.d_filtered = 0.0

        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_u = 0.0
        self.last_u_unsat = 0.0

    def update(self, setpoint: float, measurement: float) -> float:
        """Calculate control force u given target setpoint and current measurement.

        Returns:
            u: Actuator force (clipped to [u_min, u_max]) in Newtons.
        """
        error = setpoint - measurement

        # 1. Proportional term
        p_term = self.kp * error

        # 2. Derivative term
        if self.d_on == "measurement":
            if self.prev_meas is None:
                d_raw = 0.0
            else:
                d_raw = -(measurement - self.prev_meas) / self.dt
        elif self.d_on == "error":
            if self.prev_error is None:
                d_raw = error / self.dt
            else:
                d_raw = (error - self.prev_error) / self.dt
        else:
            raise ValueError(f"Unknown d_on mode: '{self.d_on}'. Choose 'measurement' or 'error'.")

        # Derivative low-pass filter: d_f += alpha * (d_raw - d_f), alpha = dt / (tau + dt)
        if self.d_filter_tau > 0.0:
            alpha = self.dt / (self.d_filter_tau + self.dt)
            if self.prev_meas is None and self.prev_error is None:
                self.d_filtered = d_raw
            else:
                self.d_filtered += alpha * (d_raw - self.d_filtered)
            d_term = self.kd * self.d_filtered
        else:
            d_term = self.kd * d_raw

        # 3. Integral term & Anti-windup clamping
        candidate_integral = self.integral + error * self.dt
        candidate_i_term = self.ki * candidate_integral

        # Unconstrained candidate control output
        u_unsat = p_term + candidate_i_term + d_term

        if self.anti_windup == "clamp":
            # Clamping: stop accumulating if saturated and error pushes further into saturation
            saturating_high = (u_unsat > self.u_max) and (error > 0.0)
            saturating_low = (u_unsat < self.u_min) and (error < 0.0)

            if not (saturating_high or saturating_low):
                self.integral = candidate_integral
            i_term = self.ki * self.integral

        elif self.anti_windup == "none":
            self.integral = candidate_integral
            i_term = candidate_i_term

        elif self.anti_windup == "backcalc":
            u_clipped = float(np.clip(u_unsat, self.u_min, self.u_max))
            tt = self.kp if self.kp > 0 else 1.0
            self.integral += (error + (u_clipped - u_unsat) / tt) * self.dt
            i_term = self.ki * self.integral

        else:
            raise ValueError(f"Unknown anti_windup mode: '{self.anti_windup}'.")

        # Compute final saturated output
        u_final_unsat = p_term + i_term + d_term
        u_sat = float(np.clip(u_final_unsat, self.u_min, self.u_max))

        # Store state for next cycle and diagnostics
        self.prev_meas = measurement
        self.prev_error = error

        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term
        self.last_u = u_sat
        self.last_u_unsat = u_final_unsat

        return u_sat

