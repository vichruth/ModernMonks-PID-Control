# Section 7(d): Slow Sampling Instability Threshold Analysis

## Summary of Measured Thresholds
- **Strict Settling Criterion (30 s horizon, last 5 s max |y - 1| < 0.05 m):** **60.5 ms**
- **Medium Settling Criterion (10 s horizon, last 2 s max |y - 1| < 0.05 m):** **59.5 ms**
- **Short-Term Check (3 s horizon peak < 2.5 m):** **66.5 ms** (a 3 s test overestimates the limit)

## Stability Regimes Over a 60 s Run
The stability limit lies in the range **59.5 ms to 60.5 ms** for convergence within ±5% error.
Because this closed loop is linear with no saturation, losing stability above 60 ms produces slowly growing oscillation rather than a limit cycle.
Measured labels from a 60 s run: **stable up to 60 ms, growing at 62 ms, exploding from 64 ms**.

## Physical Mechanism & Figure Caption
> **Figure 14 Caption:** Controller Sampling Rate Degradation under Zero-Order Hold (ZOH).
> Stepping the plant at $\Delta t_{\text{sim}} = 1$ ms while evaluating the PID controller every $\Delta t_{\text{ctrl}}$
> introduces effective phase delay $\theta_{\text{lag}} \approx \omega \cdot \frac{\Delta t_{\text{ctrl}}}{2}$.
> The system response remains nearly identical to continuous time for $\Delta t_{\text{ctrl}} \le 20$ ms (overshoot shifts slightly from 4.27% at 1 ms to 4.03% at 20 ms).
> At $\Delta t_{\text{ctrl}} = 50$ ms, noticeable phase lag increases overshoot to 11.88%.
> The critical stability boundary occurs at **$\Delta t_{crit} \approx 60.5$ ms** (30 s horizon, 5% settling band); over a 60 s run the response is stable up to 60 ms, growing at 62 ms, and exploding from 64 ms.
> A 3 s test overestimates the limit at 66.5 ms because slowly growing oscillations take longer than 3 s to diverge violently, while updates at 90–100 ms explode immediately.
