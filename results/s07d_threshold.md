# Section 7(d): Slow Sampling Instability Threshold Analysis

## Summary of Measured Thresholds
- **Strict Settling Criterion (30 s horizon, last 5 s max |y - 1| < 0.05 m):** **60.5 ms**
- **Medium Settling Criterion (10 s horizon, last 2 s max |y - 1| < 0.05 m):** **59.5 ms**
- **Short-Term Peak Divergence Criterion (3 s horizon peak < 2.5 m):** **66.5 ms**

## Stability Threshold Range
The stability limit lies in the range **59.5 ms to 60.5 ms** for asymptotic convergence within ±5% error.
If stability is defined as preventing immediate explosive divergence within the first 3 seconds, the threshold extends to **66.5 ms**.

## Physical Mechanism & Figure Caption
> **Figure 14 Caption:** Controller Sampling Rate Degradation under Zero-Order Hold (ZOH).
> Stepping the plant at $\Delta t_{\text{sim}} = 1$ ms while evaluating the PID controller every $\Delta t_{\text{ctrl}}$
> introduces effective phase delay $\theta_{\text{lag}} \approx \omega \cdot \frac{\Delta t_{\text{ctrl}}}{2}$.
> The system response remains virtually indistinguishable from continuous time for $\Delta t_{\text{ctrl}} \le 20$ ms (overshoot increases only from 4.3% to 6.8%).
> At $\Delta t_{\text{ctrl}} = 50$ ms, noticeable phase lag increases overshoot to 17.5%.
> The critical stability boundary occurs at **$\Delta t_{crit} \approx 60.5$ ms** (30 s horizon, 5% settling band).
> Above 66.5 ms, phase margin drops to zero, and updates at 90–100 ms cause violent numerical and physical divergence.
