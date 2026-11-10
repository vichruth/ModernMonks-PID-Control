# PID Control: Technical Assignment & Engineering Lab

**Role:** Physical AI Research Intern  
**Company:** Modern Monk Labs  
**Candidate Submission:** Vichruth (submission to `hr@modernmonklabs.com`)  

---

## 1. Quick Start: One Command to Run Everything

The entire simulation suite, assertion checks, 17 publication figures, and 6 benchmark tables can be reproduced with a single command from repository root:

```bash
# 1. Setup clean environment (Python 3.10+ with NumPy and Matplotlib only)
pip install -r requirements.txt

# 2. Run master pipeline (executes test_sanity.py, regenerates all figures & tables)
python3 run_all.py
```

To run individual what-if interview experiments from the command line:
```bash
python3 playground.py --kp 300 --ki 300 --kd 30 --mass 2.0 --umax 30 --no-antiwindup --kd-tau 0.02
```

Key project documentation:
- [`WRITEUP.md`](WRITEUP.md): 1-page engineering report detailing our first-principles approach, what worked, the 3 recovered bugs, and future directions.
- [`AI_USAGE.md`](AI_USAGE.md): Transparent disclosure of AI pairing and independent verification.
- [`notes/prediction_log.md`](notes/prediction_log.md): Pre-interview what-if prediction practice sheet with blank intuition entries vs simulated data.

---

## 2. Complete Map: Scripts to Figures and Results Tables

Every graph in the report is 100% reproducible from self-contained scripts in `experiments/`:

| Script | Generated Figures | Generated Tables / Data | Topic & Key Finding |
|---|---|---|---|
| `experiments/s01_loop_diagram.py` | `figures/fig01_loop_diagram.png` | — | Section 1: Closed-loop architecture & shower temperature analogy. |
| `experiments/s02_pid_terms.py` | `figures/fig02_pid_terms.png` | — | Section 2: Continuous & discrete PID error, separate P/I/D forces, and total effort. |
| `experiments/s03_plant.py` | `figures/fig03_open_loop_plant.png` | — | Section 3: Open-loop step responses ($u=0, 10, 20\text{ N}$) proving $20\text{ N}$ holding force and $\zeta = 1.118$. |
| `experiments/s04_metrics.py` | `figures/fig04_step_metrics.png` | — | Section 4: Performance criteria diagram marking rise time, peak overshoot, 2% settling band, and steady-state error. |
| `experiments/s05_each_part.py` | `figures/fig05_p_only.png`<br>`figures/fig06_p_theory_vs_sim.png`<br>`figures/fig07_add_i.png`<br>`figures/fig08_add_d.png` | `results/t01_p_metrics.md`<br>`results/t02_theory_vs_sim.md` | Section 5: P, I, D term isolation sweeps. Proves $e_{ss} = \frac{k}{K_p + k}$ (matches to 6 decimals) and critical damping at $K_d \approx 25.8\text{ N}\cdot\text{s/m}$. |
| `experiments/s06_compare.py` | `figures/fig09_p_pi_pd_pid.png`<br>`figures/fig10_final_gains.png` | `results/t03_compare_controllers.md` | Section 6: Direct comparison across P, PI, PD, and PID. Baseline ($300/300/30$) meets all criteria; grid search evaluates faster alternatives. |
| `experiments/s07_real_world.py` | `figures/fig11_windup_clamping.png`<br>`figures/fig12_noise_filtering.png`<br>`figures/fig13_setpoint_kick.png`<br>`figures/fig14_slow_sampling.png` | `results/t04_real_world_fixes.md`<br>`results/s07d_threshold.md` | Section 7: Four real-world hazards: windup clamping ($17.6\% \to 0\%$), noise filtering ($43.4\text{ N} \to 1.76\text{ N}$), kick elimination ($30,300\text{ N} \to 300\text{ N}$), and slow sampling stability ($59.5\text{--}60.5\text{ ms}$ threshold). |
| `experiments/s08_tuning.py` | `figures/fig15_zn_sustained_oscillation.png`<br>`figures/fig16_zn_vs_improved.png` | `results/t05_ziegler_nichols.md`<br>`results/s08_manual_tuning.md` | Section 8: Ziegler-Nichols on $G(s)=1/(s+1)^3$ ($K_u=8.0, T_u=3.63\text{ s}$). Improved gains cut ZN overshoot from $52.4\% \to 3.1\%$ and settle in $8.67\text{ s}$. |
| `experiments/s09_final_demo.py` | `figures/fig17_final_demo.png` | `results/t06_final_demo.md` | Section 9: Production PID verification under $30\text{ N}$ motor saturation (zero overshoot, $0.99\text{ s}$ settling, $0.0000\text{ m}$ steady error). |

---

## 3. Design Decisions & Engineering Rationale

In accordance with Section 3 of `PLAN.md`, each technical decision was made with explicit physical trade-offs:

1. **Simulator Method (Semi-Implicit Euler, $\Delta t = 1\text{ ms}$):** Symplectic Euler preserves mechanical energy and phase space volume in second-order oscillatory systems, avoiding artificial numerical energy dissipation or explosion. Halving $\Delta t$ to $0.5\text{ ms}$ changes overshoot by only $0.01\%$.
2. **PID Formulation (Parallel Form: $u = K_p e + K_i \int e + K_d \dot{e}$):** Directly matches the assignment parameter definitions ($K_p, K_i, K_d$) while maintaining clean analytical conversion to standard form ($T_i = K_p/K_i, T_d = K_d/K_p$).
3. **Derivative Input (Derivative on Measurement: $-dy/dt$):** Measuring sensor rate eliminates the mathematical impulse of step setpoint changes, reducing initial spike from $30,300\text{ N}$ down to $300\text{ N}$ (100x reduction).
4. **Derivative Filter (First-Order Low-Pass: $\tau = 0.02\text{ s}$):** Suppresses $96\%$ of actuator chatter caused by discrete differentiation of sensor noise (`Kd*sqrt(2)*sigma/dt = 30 * 1.414 = 42.4 N`) at the acceptable cost of a minor $20\text{ ms}$ phase lag.
5. **Anti-Windup (Conditional Integration Clamping):** Freezes integrator accumulation whenever the control demand exceeds motor bounds ($|u| > 30\text{ N}$) and the error acts to drive the actuator deeper into saturation.
6. **Noise Model (Gaussian Noise $\sigma = 1\text{ mm}$, seed 42):** Accurately models physical sensor quantization and thermal noise with reproducible pseudo-random seeding.
7. **Multi-Rate Zero-Order Hold (ZOH):** Models real digital microcontroller architectures by stepping the continuous physical plant at $1\text{ ms}$ while sampling and updating the controller at discrete intervals ($\Delta t_{\text{ctrl}} = 1\text{ to }100\text{ ms}$).
8. **Rise Time Definition (10% to 90% of setpoint):** Standard industrial convention; honestly reported as `"n/a (never reaches 90%)"` for weak proportional gains ($K_p \le 50$) rather than faking numbers.
9. **Overshoot Definition (Relative to Target Setpoint):** Defined strictly as $(y_{\text{peak}} - r) / r \times 100\%$ relative to target $r = 1.0\text{ m}$. Textbook formulas relative to final steady state $x_{ss}$ are explicitly converted via $\left(\frac{K_p}{k+K_p}\right)(1 + \text{OS}_{\text{final}}) - 1$.
10. **Settling Time Definition (2% of Setpoint Band):** Evaluates the final exit time outside $[0.98\text{ m}, 1.02\text{ m}]$; reported as `"never settles"` for steady-state offset responses.
11. **Steady-State Error Evaluation ($T = 15.0\text{ s}$ Horizon):** Simulations run for at least $15\text{ s}$ so that slow integrator dynamics fully converge to authentic steady-state ($e_{ss} \equiv 0.0000\text{ m}$), eliminating short-run truncation artifacts.
12. **Gain Tuning Selection:** Baseline ($K_p=300, K_i=300, K_d=30$) chosen as a simple baseline that meets all criteria ($4.27\% < 5\%$ overshoot, $1.42\text{ s} < 1.5\text{ s}$ settling, $300\text{ N}$ peak force); the grid search found faster settings such as 300/200/20 (0.46% OS, 0.243 s).
13. **Standardized Publication Styling:** Color-coded curves across all figures (P=blue `#1f77b4`, I=green `#2ca02c`, D=red `#d62728`, target=black dashed) exported at 300 DPI via headless `Agg` backend.

---

## 4. Deliverable Verification & Status

- **PDF Textbook Guide (Deliverable 1):** The official 15-page document is compiled and located at [`report/PID_Control_Guide.pdf`](report/PID_Control_Guide.pdf). It strictly adheres to the 15-page budget, incorporates all 17 publication figures (F01–F17), 6 tables, full docstrings, 5 robotics case studies, and 5 debunked engineering myths.
- **Pure Python Codebase (Deliverable 2):** Fully reproducible from scratch without external control libraries (`scipy`, `control`, `simple-pid`). Recreated via `python3 run_all.py` in about 41 s.
- **Engineering Write-Up (Deliverable 3):** Available in [`WRITEUP.md`](WRITEUP.md), detailing the first-principles approach, what worked, the 3 recovered bugs, and future directions.
- **Interview Preparation (Deliverable 4):** Prediction worksheet populated in [`notes/prediction_log.md`](notes/prediction_log.md) covering 13 physical what-if cases with blank candidate prediction slots vs simulated data.
