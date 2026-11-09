# Table 4: Real-World Engineering Hazards and Solutions

Evaluation on the baseline PID controller ($K_p = 300, K_i = 300, K_d = 30$) under practical hardware constraints.

| Problem / Hazard | Primary Metric | Before Engineering Fix | After Engineering Fix | Physical Mechanism |
|---|---|---|---|---|
| **Integrator Windup (|u| ≤ 30 N)** | Overshoot / Settling | OS: 17.6%, Settling: 2.98 s | **OS: 0.0%, Settling: 1.06 s** | Conditional clamping freezes integrator during saturation if error increases saturation. |
| **Sensor Noise (σ = 1 mm)** | Control Force Chatter std(u) | Raw D: std(u) = 43.35 N (Theory: 42.4 N) | **Filtered (τ=0.02s): 1.76 N | (τ=0.05s): 0.90 N** | First-order low-pass filter (α = dt / (τ + dt)) attenuates high-frequency noise. |
| **Setpoint Kick at t=0** | Initial Force Spike u(0) | D on error: 30,300 N spike | **D on measurement: 300 N (100x reduction)** | Differentiating y(t) instead of e(t) eliminates impulsive derivative of step input. |
| **Slow Controller Sampling** | Stability Limit Δt_crit | 100 ms update: Diverges / violent instability | **Stable for Δt ≤ 60.5 ms (Range: 59-60 ms; identical up to 20 ms)** | Zero-order hold introduces phase lag ~ω·dt/2, reducing phase margin to zero. |
