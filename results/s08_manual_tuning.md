# Section 8: Systematic Manual Tuning Recipe

Empirical impact of independently raising controller gains on the physical test system
($m = 1.0\text{ kg}, c = 10.0\text{ N}\cdot\text{s/m}, k = 20.0\text{ N/m}$, Setpoint $r = 1.0\text{ m}$).

| Parameter | Effect on Rise Time | Effect on Overshoot | Effect on Settling Time | Effect on Steady-State Error | Effect on Noise Sensitivity | Measured Evidence from Test System |
|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **Increasing $K_p$** | **Decreases** (faster initial response) | **Increases** (drops closed-loop damping $\zeta$) | **Degrades / Prolongs** (oscillations bounce) | **Decreases** (scales as $e_{ss} = \frac{k}{K_p + k}$) | **Minimal direct chatter**, but increases initial force spike $u(0) = K_p \cdot r$ | At $K_p=100$: $t_r = 0.18\text{ s}, e_{ss} = 0.167\text{ m}$. At $K_p=1000$: $t_r = 0.036\text{ s}, \text{OS} = 57.6\%$, spike = $1000\text{ N}$. |
| **Increasing $K_i$** | **Slight decrease** (accelerates toward target) | **Increases** (accumulates integrator lag / windup) | **Improves initially**, then **degrades** if too high | **Eliminates entirely** (drives $e_{ss} \to 0.000\text{ m}$) | **Zero chatter** (integrator is a low-pass filter $1/s$) | $K_i=50 \to \text{OS } 0.0\%$. $K_i=200 \to \text{OS } 21.9\%$. Settles holding force to exactly $20.00\text{ N}$. |
| **Increasing $K_d$** | **Increases** (dynamic braking resists motion) | **Decreases dramatically** (quenches bouncing) | **Improves up to $\zeta=1.0$**, then sluggish | **No effect** ($\dot{x} = 0$ at rest) | **Severely increases chatter** ($\text{std}(u) \propto K_d \frac{\sqrt{2}\sigma}{\Delta t}$) | At $K_p=300$: $K_d=0 \to \text{OS } 31.2\%$. $K_d=10 \to \text{OS } 4.85\%$. $K_d=30 \to \text{OS } 0.0\%$. Unfiltered noise chatter = $43.4\text{ N}$. |

## Step-by-Step Practical Tuning Procedure

1. **(1) Set $K_i = K_d = 0$:** Start with proportional-only control ($K_p = 0, K_i = 0, K_d = 0$) to establish the baseline open-loop response.
2. **(2) Raise $K_p$ until it's fast but starts to overshoot:** Increase $K_p$ to achieve acceptable responsiveness (e.g., $K_p = 300\text{ N/m}$). Notice that steady-state error shrinks ($e_{ss} = 0.0625\text{ m}$), but the system begins to bounce with $31.2\%$ overshoot.
3. **(3) Add $K_d$ until the bounce is damped:** Increase derivative gain (e.g., $K_d = 30\text{ N}\cdot\text{s/m}$) to introduce dynamic velocity braking. This raises closed-loop damping ($\zeta = 1.12$), completely quenching the oscillation and eliminating overshoot.
4. **(4) Add $K_i$ until steady error is gone:** Introduce integral gain (e.g., $K_i = 300\text{ N/(m}\cdot\text{s)}$) to accumulate the required $20\text{ N}$ spring holding force, driving steady-state position to exactly $1.0000\text{ m}$ ($e_{ss} \equiv 0.0000\text{ m}$).
5. **(5) If overshoot returns, lower $K_i$ or raise $K_d$:** Adding integral action introduces phase lag that can cause a secondary overshoot ($4.27\%$). If overshoot exceeds tolerance, trim $K_i$ or slightly increase $K_d$ to restore critical damping.
6. **(6) Re-check with noise and force limit:** Test the tuned controller under realistic hardware conditions: enable the actuator force limit ($|u| \le 30\text{ N}$) with clamping anti-windup, and enable sensor noise ($\sigma = 1\text{ mm}$) with a first-order low-pass derivative filter ($\tau = 0.02\text{ s}$) to prevent actuator chatter.
