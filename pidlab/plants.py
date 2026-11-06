"""Plant models for PID control simulation.

Plants implement:
  - step(u, dt) -> float: updates internal state and returns plant output
  - reset(...) -> None: resets state to initial conditions
  - output -> float: property returning current output
"""

class MassSpringDamper:
    """Second-order mass-spring-damper mechanical system.

    Equation of motion:
        m * x''(t) + c * x'(t) + k * x(t) = u(t)
    where:
        m: mass (kg)
        c: damping coefficient (N*s/m)
        k: spring stiffness (N/m)
        u: applied force (N)
        x: displacement / position (m)
    """

    def __init__(self, m: float = 1.0, c: float = 10.0, k: float = 20.0,
                 x0: float = 0.0, v0: float = 0.0):
        self.m = float(m)
        self.c = float(c)
        self.k = float(k)
        self.x = float(x0)
        self.v = float(v0)

    def reset(self, x0: float = 0.0, v0: float = 0.0) -> None:
        """Reset state displacement and velocity."""
        self.x = float(x0)
        self.v = float(v0)

    def step(self, u: float, dt: float) -> float:
        """Advance the plant state by dt using semi-implicit (symplectic) Euler.

        Semi-implicit Euler updates velocity using current acceleration,
        then updates position using the newly calculated velocity:
            a = (u - c*v - k*x) / m
            v_{k+1} = v_k + a * dt
            x_{k+1} = x_k + v_{k+1} * dt
        """
        a = (u - self.c * self.v - self.k * self.x) / self.m
        self.v += a * dt
        self.x += self.v * dt
        return self.x

    @property
    def output(self) -> float:
        """Current position displacement x (m)."""
        return self.x


class ThreeLags:
    """Three identical cascaded first-order lags: G(s) = 1 / (s + 1)^3.

    State representation:
        dx1/dt = u - x1
        dx2/dt = x1 - x2
        dx3/dt = x2 - x3
        y = x3

    Theoretical properties for Ziegler-Nichols tuning:
        At critical frequency omega_u = sqrt(3) rad/s:
            Phase = -3 * arctan(sqrt(3)) = -180 deg
            |G(j*omega_u)| = 1 / (1 + 3)^(3/2) = 1/8
            Ultimate gain Ku = 8.0
            Ultimate period Tu = 2*pi / sqrt(3) ~= 3.628 s
    """

    def __init__(self, x0: float = 0.0):
        self.x1 = float(x0)
        self.x2 = float(x0)
        self.x3 = float(x0)

    def reset(self, x0: float = 0.0) -> None:
        """Reset internal lag filter states."""
        self.x1 = float(x0)
        self.x2 = float(x0)
        self.x3 = float(x0)

    def step(self, u: float, dt: float) -> float:
        """Advance lag states by dt using Euler integration."""
        self.x1 += (u - self.x1) * dt
        self.x2 += (self.x1 - self.x2) * dt
        self.x3 += (self.x2 - self.x3) * dt
        return self.x3

    @property
    def output(self) -> float:
        """Current output y = x3."""
        return self.x3
