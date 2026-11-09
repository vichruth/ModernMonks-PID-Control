# Project Write-Up: Physical AI Control Lab

**Role:** Physical AI Research Intern Applicant  
**Company:** Modern Monk Labs  
**Candidate:** Vichruth (`hr@modernmonklabs.com`)  
**Repository:** Pure Python (NumPy + Matplotlib only; zero external control libraries)

---

## 1. Approach & Philosophy

Our objective was to build a first-principles, industrial-grade simulation and pedagogical framework for PID control from scratch. Rather than relying on black-box control toolboxes (`scipy.signal`, `control`, or `simple-pid`), every equation of motion, numerical integrator, discrete controller term, and filtering algorithm was derived analytically and implemented natively.

Key architectural decisions include:
- **Symplectic Physics Simulation:** Used semi-implicit Euler integration ($\Delta t = 1\text{ ms}$) for the second-order mechanical plant ($m \ddot{x} + c \dot{x} + k x = u$). Unlike explicit Forward Euler (which artificially injects energy into oscillatory systems) or complex implicit solvers, symplectic Euler guarantees phase-space volume preservation and strict long-term numerical energy conservation.
- **Hardware-Informed Digital Control:** Implemented multi-rate execution, where the physical plant advances at continuous-scale $1\text{ ms}$ steps while the controller samples and commands via Zero-Order Hold (ZOH) across configurable digital sample times ($\Delta t_{\text{ctrl}} = 1\text{ to }100\text{ ms}$).
- **Defensive Production Architecture:** The reusable `PID` class incorporates three essential physical hardening mechanisms:
  1. *Conditional integration clamping* to prevent actuator windup during force saturation ($|u| \le 30\text{ N}$).
  2. *Derivative-on-measurement* ($-dy/dt$) to eliminate high-frequency setpoint kick impulses.
  3. *First-order low-pass filtering* ($\tau = 0.02\text{ s}$) to prevent high-frequency actuator chatter induced by sensor noise.

---

## 2. What Worked

1. **Exact Theory-to-Simulation Alignment:**
   - Proportional-only steady-state errors matched the analytical prediction $e_{ss} = \frac{k}{K_p + k}$ to 6 decimal places across all gains ($K_p = 10, 50, 100, 300, 1000$).
   - Closed-loop damping ratio analysis correctly predicted critical damping at $K_d = 2\sqrt{m(k+K_p)} - c \approx 25.8\text{ N}\cdot\text{s/m}$ (for $K_p = 300$). Overshoot completely vanished for $K_d \ge 25.8$.
   - Ziegler-Nichols frequency response analysis for $G(s) = 1/(s+1)^3$ yielded exact theoretical values: ultimate frequency $\omega_u = \sqrt{3}\text{ rad/s}$, ultimate gain $K_u = 8.00$ (theory: 8.00), and ultimate period $T_u = 3.625\text{ s}$ (theory: $2\pi/\sqrt{3} \approx 3.628\text{ s}$).
2. **Robust Industrial Hardening:**
   - Anti-windup clamping reduced overshoot from $17.6\%$ down to $0.0\%$ under a $30\text{ N}$ motor limit, cutting settling time from $2.98\text{ s}$ to $1.06\text{ s}$.
   - D-on-measurement eliminated the $30,300\text{ N}$ step-derivative impulse, capping initial demand at an orderly $300\text{ N}$ (a 100x reduction).
   - Low-pass derivative filtering suppressed sensor noise chatter from $43.35\text{ N}$ RMS down to $1.76\text{ N}$ RMS, saving actuators from acoustic buzzing and thermal burnout.
3. **Automated Sanity Testing:**
   - A 10-test automated verification suite (`tests/test_sanity.py`) enforces strict tolerances on physical metrics, anti-windup behavior, noise suppression, and gain calibrations on every run.

---

## 3. What Didn't Work & Recovered Bugs

During systematic benchmarking, three critical discrepancies were uncovered and corrected:

1. **Overshoot Reference Definition Bug:**
   - *Issue:* In Section 5, $K_p = 300, K_d = 10$ initially reported an overshoot of $11.75\%$, deviating from the textbook damping formula prediction of $5.0\%$.
   - *Root Cause:* The analytical second-order formula $\exp\left(-\frac{\pi\zeta}{\sqrt{1-\zeta^2}}\right)$ evaluates peak overshoot relative to the final steady-state value $x_{ss} = \frac{K_p}{k+K_p} = 0.9375\text{ m}$. However, our global metric standard evaluates overshoot relative to the target setpoint ($r = 1.0\text{ m}$).
   - *Resolution:* Converted the analytical prediction via $\text{OS}_{\text{setpoint}} = \left[\left(\frac{K_p}{k+K_p}\right)(1 + \text{OS}_{\text{final}}) - 1\right] \times 100\%$. This gave $5.02\%$ theory vs $4.85\%$ simulation, reconciling the gap.
2. **Simulation Horizon Truncation Artifact:**
   - *Issue:* Baseline PID ($300/300/30$) reported a small steady-state error of $-0.0023\text{ m}$ under an initial 3.5 s run horizon.
   - *Root Cause:* The continuous spring pull ($20\text{ N}$) requires the integrator to slowly accumulate holding force. Truncating at 3.5 s caught the integrator during its fine asymptotic tail settling.
   - *Resolution:* Increased default simulation horizon to $T = 15.0\text{ s}$ in `sim.py` and experiment scripts. The true steady-state error settled to $0.0000\text{ m}$ ($1.34 \times 10^{-8}\text{ m}$ numerical precision).
3. **Slow-Sampling Discrepancy & Stability Threshold:**
   - *Issue:* Short 3 s bisection sweeps evaluating peak divergence reported an instability threshold of $66.5\text{ ms}$, whereas longer runs showed stability loss near $60\text{ ms}$.
   - *Root Cause:* At $\Delta t_{\text{ctrl}} \in [60\text{ ms}, 66\text{ ms}]$, the system does not diverge instantly; instead, digital sampling phase lag ($\theta_{\text{lag}} \approx \omega \frac{\Delta t}{2}$) slowly destabilizes the loop into sustained or growing limit cycles over 10–30 s.
   - *Resolution:* Extended bisection horizon to $30\text{ s}$ with a strict 2% settling criterion ($\max |y - 1| < 0.05\text{ m}$ over final 5 s). Pinpointed the precise threshold range ($59.5\text{--}60.5\text{ ms}$) and documented both criteria in `results/s07d_threshold.md`.

---

## 4. What to Explore Next

With additional engineering time, three high-impact directions should be explored:
1. **Gain Scheduling & Non-Linear Mechanics:** Extend the controller to plants with non-linear spring constants ($k(x) = k_0 + k_1 x^2$) and aerodynamic drag ($F_d = \frac{1}{2}\rho C_d A v^2$), using automated gain-scheduling tables interpolated across operating points.
2. **Cascaded Multi-Loop Control for Physical Robotics:** Implement an inner angular velocity loop nested inside an outer attitude orientation loop (as used in quadrotor flight controllers and robotic joint impedance control).
3. **Hardware-In-The-Loop (HIL) & Real-Time Microcontroller Deployment:** Export the pure-Python `PID` class to C99 / Rust for microsecond-level execution on STM32 / ESP32 microcontrollers, interfacing directly with physical encoders and motor PWM drivers.
