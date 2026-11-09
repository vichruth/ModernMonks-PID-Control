# Table 5: Ziegler-Nichols and Advanced Tuning Rules Benchmark

Target Plant: $G(s) = \frac{1}{(s+1)^3}$. Measured: $K_u = 8.00$, $T_u = 3.625\text{ s}$.

| Tuning Method | Formula / Gains | $K_p / K_i / K_d$ | Overshoot (%) | Settling Time (2%) | Rise Time (s) | Evaluation & Characteristics |
|---|---|---|---|---|---|---|
| Classic ZN PID | Parallel PID | 4.80 / 2.65 / 2.18 | 52.44% | 9.898 s | 0.950 s | Fast rise (0.95 s), but severe oscillatory overshoot (52.4%); unsuitable for precision robotics. |
| Classic ZN PI | Parallel PID | 3.60 / 1.19 / 0.00 | 56.05% | 30.735 s | 1.054 s | Massive overshoot (56.1%) and prolonged settling (30.7 s); lacks derivative damping. |
| Tyreus-Luyben PID | Parallel PID | 3.64 / 0.46 / 2.09 | 0.82% | 16.870 s | 1.506 s | Conservative and calm (<1% overshoot), but sluggish tail settling (16.9 s) due to weak Ki. |
| Improved Tuned PID | Parallel PID | 3.60 / 0.80 / 3.20 | 3.10% | 8.672 s | 1.675 s | **Sweet Spot**: Meets all criteria (<10% OS: 3.10%, <10 s settling: 8.67 s, zero error). |
