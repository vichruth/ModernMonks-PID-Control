# Table 2: Analytical Theory vs Numerical Simulation Benchmark

### Part A: P-Only Controller ($K_p \in [10, 1000]$)
Comparison of steady-state error $e_{ss} = \frac{k}{K_p + k}$ and damping ratio $\zeta = \frac{c}{2\sqrt{m(k+K_p)}}$.

| $K_p$ | Theory $e_{ss}$ | Sim $e_{ss}$ | Error Diff | Damping $\zeta$ | Theory Overshoot (vs Setpoint) | Sim Overshoot (vs Setpoint) |
|---|---|---|---|---|---|---|
| 10 | 0.6667 | 0.6667 | 0.000000 | 0.913 | 0.00% | 0.00% |
| 50 | 0.2857 | 0.2857 | 0.000000 | 0.598 | 0.00% | 0.00% |
| 100 | 0.1667 | 0.1667 | 0.000000 | 0.456 | 0.00% | 0.00% |
| 300 | 0.0625 | 0.0625 | 0.000000 | 0.280 | 31.32% | 31.22% |
| 1000 | 0.0196 | 0.0196 | 0.000000 | 0.157 | 57.62% | 57.56% |

### Part B: Adding Derivative Action ($K_p = 300, K_d \in [0, 60]$)
Closed-loop characteristic equation: $m s^2 + (c + K_d) s + (k + K_p) = 0$.
Closed-loop damping ratio: $\zeta = \frac{c + K_d}{2\sqrt{m(k + K_p)}}$.

> **Important Note on Overshoot Definition:**
> The textbook formula $\text{OS}_{\text{final}} = \exp\left(-\frac{\pi \zeta}{\sqrt{1-\zeta^2}}\right)$ defines peak overshoot relative to the **final steady-state position** $x_{ss} = \frac{K_p}{k + K_p}$.
> For $K_p = 300, k = 20$, $x_{ss} = 300/320 = 0.9375$ m.
> In project specifications, overshoot is defined strictly relative to the **target setpoint** ($r = 1.0$ m):
> $$\text{OS}_{\text{setpoint}} = \left[\frac{x_{\text{peak}} - r}{r}\right] \times 100\% = \left[\left(\frac{K_p}{k+K_p}\right)(1 + \text{OS}_{\text{final}}) - 1\right] \times 100\%$$
> For $K_d = 10$: $\text{OS}_{\text{final}} = 12.03\%$, which converts to $\text{OS}_{\text{setpoint}} = 5.02\%$. The simulation measures $4.85\%$, matching theory within $0.17\%$.

| $K_d$ (Ns/m) | Damping $\zeta$ | OS vs Final (Theory) | OS vs Setpoint (Theory) | OS vs Setpoint (Simulation) | Match Status |
|---|---|---|---|---|---|
| 0 | 0.280 | 40.07% | 31.32% | 31.22% | Matches (<0.2%) |
| 10 | 0.559 | 12.03% | 5.02% | 4.85% | Matches (<0.2%) |
| 30 | 1.118 | 0.00% | 0.00% | 0.00% | Matches (<0.2%) |
| 60 | 1.957 | 0.00% | 0.00% | 0.00% | Matches (<0.2%) |
