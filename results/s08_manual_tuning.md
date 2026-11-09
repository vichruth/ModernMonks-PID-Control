# Section 8: Systematic Manual Tuning Recipe

Empirical impact of independently raising controller gains on the physical test system
($m = 1.0\text{ kg}, c = 10.0\text{ N}\cdot\text{s/m}, k = 20.0\text{ N/m}$, Setpoint $r = 1.0\text{ m}$).

| Parameter | Effect on Rise Time | Effect on Overshoot | Effect on Settling Time | Effect on Steady-State Error | Effect on Noise Sensitivity | Measured Evidence from Test System |
|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **Increasing $K_p$** | **Decreases** (faster initial response) | **Increases** (drops closed-loop damping $\zeta$) | **Degrades / Prolongs** (oscillations bounce) | **Decreases** (scales as $e_{ss} = \frac{k}{K_p + k}$) | **Minimal direct chatter**, but increases initial force spike $u(0) = K_p \cdot r$ | At $K_p=100$: $t_r = 0.18\text{ s}, e_{ss} = 0.167\text{ m}$. At $K_p=1000$: $t_r = 0.036\text{ s}, \text{OS} = 57.6\%$, spike = $1000\text{ N}$. |
| **Increasing $K_i$** | **Slight decrease** (accelerates toward target) | **Increases** (accumulates integrator lag / windup) | **Improves initially**, then **degrades** if too high | **Eliminates entirely** (drives $e_{ss} \to 0.000\text{ m}$) | **Zero chatter** (integrator is a low-pass filter $1/s$) | $K_i=50 \to \text{OS } 0.0\%$. $K_i=200 \to \text{OS } 21.9\%$. Settles holding force to exactly $20.00\text{ N}$. |
| **Increasing $K_d$** | **Increases** (dynamic braking resists motion) | **Decreases dramatically** (quenches bouncing) | **Improves up to $\zeta=1.0$**, then sluggish | **No effect** ($\dot{x} = 0$ at rest) | **Severely increases chatter** ($\text{std}(u) \propto K_d \frac{\sqrt{2}\sigma}{\Delta t}$) | At $K_p=300$: $K_d=0 \to \text{OS } 31.2\%$. $K_d=10 \to \text{OS } 4.85\%$. $K_d=30 \to \text{OS } 0.0\%$. Unfiltered noise chatter = $43.4\text{ N}$. |

## Step-by-Step Practical Tuning Procedure

1. **Set all gains to zero:** $K_p = 0, K_i = 0, K_d = 0$.
2. **Increase $K_p$ until responsiveness is achieved:** Raise $K_p$ until the block moves briskly toward the target setpoint (e.g., $K_p = 300$). Notice that steady-state error persists ($e_{ss} = 0.0625\text{ m}$) and oscillations emerge (overshoot = $31.2\%$).
3. **Increase $K_d$ to quench oscillation:** Add derivative damping until overshoot drops to the target limit (e.g., $K_d = 30\text{ N}\cdot\text{s/m}$ achieves $\zeta = 1.12$, eliminating overshoot). Always engage derivative low-pass filtering ($\tau = 0.02\text{ s}$) to prevent actuator chatter from sensor noise.
4. **Increase $K_i$ to eliminate steady-state offset:** Slowly raise $K_i$ (e.g., $K_i = 300\text{ N/(m}\cdot\text{s)}$) to build holding force memory, driving final position to exactly $1.0000\text{ m}$ ($e_{ss} \equiv 0.0000\text{ m}$).
5. **Enable clamping anti-windup:** Always clamp integration during actuator saturation ($|u| \le 30\text{ N}$) to prevent catastrophic overshoot caused by accumulated integrator windup.
