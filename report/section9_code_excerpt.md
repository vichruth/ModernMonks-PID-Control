# Section 9 Code Excerpt: Industrial-Grade Reusable PID Controller

Below is the self-contained, dependency-free Python implementation of our production parallel-form PID controller. It incorporates all real-world hardening features derived in Section 7: conditional integration clamping (anti-windup), derivative-on-measurement (kick suppression), and first-order low-pass derivative filtering (noise attenuation).

```python
import numpy as np

class PID:
    """Discrete Parallel-Form PID Controller with Industrial Hardening.

    Formula:
        u(t) = Kp * e(t) + Ki * integral(e(t)*dt) + Kd * de(t)/dt
    
    Units:
        e: error [m]
        u: actuator control force [N]
        Kp: proportional gain [N/m]
        Ki: integral gain [N/(m*s)]
        Kd: derivative gain [N*s/m]
        dt: discrete sampling period [s]
        tau: derivative filter time constant [s]
    """

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        dt: float = 0.001,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        anti_windup: str = "clamp",       # "clamp" | "none" | "backcalc"
        d_on: str = "measurement",        # "measurement" | "error"
        d_filter_tau: float = 0.0,        # [s]; 0.0 = unfiltered
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
        """Reset internal accumulator and diagnostic memories."""
        self.integral = 0.0
        self.prev_meas = None
        self.prev_error = None
        self.d_filtered = 0.0

        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_u = 0.0

    def update(self, setpoint: float, measurement: float) -> float:
        """Compute bounded control effort u(k) given current setpoint and sensor feedback.

        Parameters:
            setpoint: Target physical reference value r(k) [m]
            measurement: Current measured system state y(k) [m]

        Returns:
            u_sat: Actuator command force clipped to [u_min, u_max] [N]
        """
        error = setpoint - measurement

        # 1. Proportional Term (Present Error)
        p_term = self.kp * error

        # 2. Derivative Term (Rate of Change)
        if self.d_on == "measurement":
            # D on measurement avoids setpoint kick: d(-y)/dt = -dy/dt
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
            raise ValueError(f"Unknown d_on mode: '{self.d_on}'")

        # First-order low-pass filter on derivative term:
        # H(s) = 1 / (tau*s + 1) -> d_f[k] = d_f[k-1] + alpha * (d_raw - d_f[k-1])
        if self.d_filter_tau > 0.0:
            alpha = self.dt / (self.d_filter_tau + self.dt)
            if self.prev_meas is None and self.prev_error is None:
                self.d_filtered = d_raw
            else:
                self.d_filtered += alpha * (d_raw - self.d_filtered)
            d_term = self.kd * self.d_filtered
        else:
            d_term = self.kd * d_raw

        # 3. Integral Term & Anti-Windup Clamping (Past Accumulated Error)
        candidate_integral = self.integral + error * self.dt
        candidate_i_term = self.ki * candidate_integral

        # Total unconstrained control demand
        u_unsat = p_term + candidate_i_term + d_term

        if self.anti_windup == "clamp":
            # Conditional Integration: Freeze accumulator if saturated
            # AND the current error would drive the actuator further into saturation.
            sat_high = (u_unsat > self.u_max) and (error > 0.0)
            sat_low = (u_unsat < self.u_min) and (error < 0.0)

            if not (sat_high or sat_low):
                self.integral = candidate_integral
            i_term = self.ki * self.integral

        elif self.anti_windup == "none":
            self.integral = candidate_integral
            i_term = candidate_i_term
        else:
            raise ValueError(f"Unknown anti_windup mode: '{self.anti_windup}'")

        # 4. Actuator Saturation Clipping
        u_sat = float(np.clip(p_term + i_term + d_term, self.u_min, self.u_max))

        # Update memories for next discrete timestep
        self.prev_meas = measurement
        self.prev_error = error
        self.last_p = p_term
        self.last_i = i_term
        self.last_d = d_term
        self.last_u = u_sat

        return u_sat
```
