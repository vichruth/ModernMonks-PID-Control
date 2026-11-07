# Table 3: Controller Architecture Comparison and Final Gain Selection

Evaluation across P, PI, PD, and PID controllers on the 1 kg mass-spring-damper ($m=1.0$ kg, $c=10.0$ Ns/m, $k=20.0$ N/m).

| Controller Type | Gains ($K_p / K_i / K_d$) | Overshoot (%) | Settling Time (2%) | Rise Time (10-90%) | Final Error | Peak Force (N) | Practical Assessment |
|---|---|---|---|---|---|---|---|
| P (Kp=300) | 300 / 0 / 0 | 31.22% | never settles | 0.077 s | 0.0625 m | 300.00 N | Unacceptable steady error ($0.0625$ m); holding spring requires continuous error. |
| PI (Kp=100, Ki=200) | 100 / 200 / 0 | 21.90% | 0.845 s | 0.148 s | 0.0001 m | 102.18 N | Zero steady error, but excessive overshoot ($21.9%$) due to lack of derivative damping. |
| PD (Kp=300, Kd=30) | 300 / 0 / 30 | 0.00% | never settles | 0.305 s | 0.0625 m | 300.00 N | Zero overshoot, but permanently stops short ($0.0625$ m); never settles within 2% band. |
| PID (300/300/30) | 300 / 300 / 30 | 4.27% | 1.422 s | 0.206 s | -0.0023 m | 300.30 N | **Optimal candidate**: < 5% overshoot, fast settling (1.42 s), zero error, peak force 300 N. |
| Aggressive PID (600/300/30) | 600 / 300 / 30 | 1.18% | 0.157 s | 0.102 s | 0.0001 m | 600.30 N | Extremely fast (0.16 s settling), but demands massive 600 N at t=0 (saturates 30 N motors). |

### Systematic Gain Optimization Grid Search

Selection Criteria:
1. **Peak Overshoot:** $< 5.0\%$ (prevent mechanical shock / bouncing)
2. **Settling Time:** $< 1.5\text{ s}$ to within $\pm 2\%$ band
3. **Steady-State Error:** $\equiv 0.00\text{ m}$ (annihilate steady error)
4. **Actuator Demand:** Minimum initial force spike $u(0) = K_p \cdot r$

| Rank | Gains ($K_p / K_i / K_d$) | Overshoot (%) | Settling Time (s) | Rise Time (s) | Peak Force (N) | Decision |
|---|---|---|---|---|---|---|
| 1 | 400 / 200 / 20 | 1.92% | 0.171 s | 0.114 s | 400.2 N | Viable alternative |
| 2 | 300 / 200 / 20 | 0.46% | 0.243 s | 0.154 s | 300.2 N | Viable alternative |
| 3 | 400 / 300 / 30 | 1.87% | 0.250 s | 0.158 s | 400.3 N | Viable alternative |
| 4 | 400 / 200 / 30 | 0.00% | 0.289 s | 0.166 s | 400.2 N | Viable alternative |
| 5 | 400 / 300 / 20 | 3.67% | 0.306 s | 0.111 s | 400.3 N | Viable alternative |
