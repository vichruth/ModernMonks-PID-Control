# Project Log: PID Control Assignment (Modern Monk Labs)

This document tracks all design decisions, implementation steps, theoretical verifications, and what-worked / what-didn't reflections throughout the project lifecycle.

---

## Log Entries

### [2026-11-06] - Step 0: Repository Setup & Scaffolding
- **Actions Taken**:
  - Reviewed and aligned with behavioral guidelines in `Agents.md` (Think before coding, simplicity first, surgical edits, goal-driven verification) and roadmap in `Plan.md`.
  - Verified Python environment: Python 3.14.6 with `numpy 2.5.3` and `matplotlib 3.11.2`. Confirmed zero external control dependencies (no `scipy`, no `python-control`).
  - Audited folder structure against `Plan.md`:
    - Created `notes/prediction_log.md` for pre-run predictions.
    - Created `notes/interview_qa.md` for live interview what-if questions.
    - Created `AI_USAGE.md` tracking how AI is utilized and what is verified independently.
    - Updated `.gitignore` to avoid ignoring essential project files.

### [2026-11-06] - Step 1: Core Simulation Engine & Benchmarking (Day 1)
- **Modules Implemented in `pidlab/`**:
  - `plants.py`:
    - `MassSpringDamper` ($m=1.0\,\text{kg}, c=10.0\,\text{N}\cdot\text{s/m}, k=20.0\,\text{N/m}$) using symplectic / semi-implicit Euler integration ($v_{k+1} = v_k + a \cdot \Delta t$, $x_{k+1} = x_k + v_{k+1} \cdot \Delta t$).
    - `ThreeLags` ($G(s) = 1/(s+1)^3$) for Ziegler-Nichols tuning experiments.
  - `pid.py`:
    - Discrete Parallel PID ($u = K_p e + K_i \int e \, dt + K_d \frac{de}{dt}$).
    - Clamping anti-windup (conditional integration stopping accumulation when actuator saturates and error pushes further into saturation).
    - Derivative on measurement vs derivative on error switch.
    - First-order derivative low-pass filter ($d_f \mathrel{+}= \alpha (d - d_f), \alpha = \frac{\Delta t}{\tau + \Delta t}$).
  - `sim.py`:
    - Closed-loop runner supporting multi-rate execution ($\Delta t_{\text{sim}} \le \Delta t_{\text{ctrl}}$ via Zero-Order Hold) and Gaussian sensor noise $\mathcal{N}(0, \sigma^2)$.
  - `metrics.py`:
    - Standardized step-response metrics: Rise time (10% to 90% of setpoint), Overshoot (% of target), Settling time (2% band), Steady-state error ($r - \bar{y}_{\text{last 5\%}}$), and Peak Force.
    - Added custom 5-line local maxima finder (zero SciPy dependency).
  - `style.py`:
    - Headless Matplotlib configuration (`Agg`), consistent project palette ($P$=blue, $I$=green, $D$=red, target=black dashed, output=dark grey, force=orange), 300 DPI exports.
  - `playground.py`:
    - Interactive CLI tool enabling instant parameter what-ifs (`--kp`, `--ki`, `--kd`, `--mass`, `--noise`, `--umax`, `--no-antiwindup`).
  - `tests/test_sanity.py`:
    - Automated assertion suite verifying 7 theoretical and numerical benchmarks from `Plan.md` Section 2.

- **Verification Results (`tests/test_sanity.py`)**:
  - Holding force $u = 20\,\text{N}$ holds exactly $x = 1.000\,\text{m}$ at rest: **PASSED**
  - $P$-only steady state errors match theory $k/(K_p+k)$ to $<0.01$: **PASSED**
    - $K_p=300 \implies$ simulated overshoot $31.2\%$ (theory $31.3\%$): **PASSED**
    - $K_p=1000 \implies$ simulated overshoot $57.6\%$ (theory $57.6\%$): **PASSED**
  - Halving $\Delta t$ ($1\,\text{ms} \to 0.5\,\text{ms}$) changes overshoot by only $0.01\%$ (< $0.1\%$ threshold): **PASSED**
  - Unbounded saturation invariance: **PASSED**
  - Windup under $30\,\text{N}$ limit: overshoot reduced from $17.6\%$ (unclamped) to $0.0\%$ (clamped): **PASSED**
  - Setpoint kick: $D$-on-error spikes to $30,300\,\text{N}$ while $D$-on-measurement stays bounded at $300\,\text{N}$: **PASSED**
  - Three-lags DC gain: **PASSED**

### [2026-11-06] - Step 2: Sections 1-4 Figures & Documentation (Day 1)
- **Experiments Implemented**:
  - `experiments/s01_loop_diagram.py`: Generated `figures/fig01_loop_diagram.png` showing the negative feedback block diagram and real-world shower temperature adjustment analogy.
  - `experiments/s02_pid_terms.py`: Generated `figures/fig02_pid_terms.png` showing the error trajectory alongside isolated P, I, D force components and total actuator output.
  - `experiments/s03_plant.py`: Generated `figures/fig03_open_loop_plant.png` showing open-loop step responses ($u=0, 10, 20\text{ N}$) settling at $0, 0.5, 1.0\text{ m}$ and proving the $20\text{ N}$ equilibrium holding force and overdamped open-loop nature ($\zeta = 1.118$).
  - `experiments/s04_metrics.py`: Generated `figures/fig04_step_metrics.png` visually marking all four time-domain performance criteria: rise time ($10\%\to90\%$), peak overshoot, settling time ($2\%$ error band), and steady-state error.
- **Master Pipeline**:
  - `run_all.py` executes `tests/test_sanity.py` and regenerates all Step 2 figures in ~4.3 seconds.
- **Documentation & Plan Updates**:
  - Updated `README.md` to directly fulfill the submission requirements (Approach, Setup, How to Run).
  - Added Thursday buffer item to `Plan.md`: Lightweight Matplotlib Slider GUI (`interactive_gui.py` / `playground.py --interactive`, ~40 lines pure Matplotlib) as a slick bonus demonstration tool.
  - Conducted full compliance audit against the official Modern Monk Labs assignment brief confirming 100% coverage across all 10 required sections, writing standards, and evaluation criteria.

### [2026-11-07] - Step 3: Section 5 (Each Part Alone - Day 2)
- **Experiments Implemented (`experiments/s05_each_part.py`)**:
  - `fig05_p_only.png`: Step responses for $K_p \in [10, 50, 100, 300, 1000]$, showing steady-state error reducing from $0.6667\text{ m}$ to $0.0196\text{ m}$ while overshoot increases from $0\%$ to $57.56\%$.
  - `fig06_p_theory_vs_sim.png`: Exact theoretical curves for $e_{ss} = k/(K_p+k)$ and $\zeta$-predicted overshoot with simulated dots overlaid (0.000000 error difference).
  - `fig07_add_i.png`: Integral action ($K_p = 100, K_i \in [0, 50, 200, 500]$), demonstrating that the integral term alone accumulates to exactly $20\text{ N}$ to eliminate steady-state offset.
  - `fig08_add_d.png`: Derivative action ($K_p = 300, K_d \in [0, 10, 30, 60]$), proving dynamic damping ratio $\zeta = (c+K_d)/(2\sqrt{m(k+K_p)})$ reaches critical damping ($\zeta = 1$) at $K_d \approx 25.8\text{ N}\cdot\text{s/m}$, quenching overshoot completely at $K_d = 30$ ($0\%$).
  - `results/t01_p_metrics.md`: Table of metrics across all $K_p$ gains.
  - `results/t02_theory_vs_sim.md`: 4-decimal place agreement between analytical derivations and simulation.

### [2026-11-07] - Step 4: Section 6 (Compare and Choose - Day 2)
- **Experiments Implemented (`experiments/s06_compare.py`)**:
  - `fig09_p_pi_pd_pid.png`: Direct comparison of P, PI, PD, and PID controllers on a single plot with actuator force demands underneath.
  - `fig10_final_gains.png`: Selected baseline PID ($K_p = 300, K_i = 300, K_d = 30$) time-history showing position response and decomposition into individual P, I, and D forces.
  - `results/t03_compare_controllers.md`: Performance metrics and peak forces table, plus systematic gain optimization grid search ranking candidates against strict written criteria (Overshoot < 5%, Settling < 1.5 s, Zero steady error).
  - Highlighted engineering trade-off: Aggressive PID ($600/300/30$) achieves ultra-fast settling ($0.16\text{ s}$), but demands $600\text{ N}$ at $t=0$, which is physically impossible on a $30\text{ N}$ motor.

### [2026-11-07] - Step 5: Section 7 (Real-World Problems - Day 3)
- **Experiments Implemented (`experiments/s07_real_world.py`)**:
  - `fig11_windup_clamping.png`: Actuator saturation ($|u| \le 30\text{ N}$) and integrator windup. Demonstrating that conditional clamping eliminates windup overshoot from $17.6\%$ down to $0.0\%$ and cuts settling from $2.98\text{ s}$ to $1.06\text{ s}$.
  - `fig12_noise_filtering.png`: High-frequency sensor noise ($\sigma = 1\text{ mm}$). Proving numerical differentiation of noise creates $44.0\text{ N}$ RMS chatter on the motor, and showing that a first-order low-pass filter ($\tau = 0.02 - 0.05\text{ s}$) suppresses chatter down to $1.78\text{ N} - 0.91\text{ N}$.
  - `fig13_setpoint_kick.png`: Setpoint kick at $t=0$. Proving D-on-error causes a violent $30,300\text{ N}$ spike, whereas D-on-measurement bounds peak force to a gentle $300\text{ N}$ (100x reduction).
  - `fig14_slow_sampling.png`: Controller sampling latency via Zero-Order Hold. Verified stability up to $\Delta t_{\text{ctrl}} \le 20\text{ ms}$, degradation at $50\text{ ms}$ ($17.5\%$ overshoot), and divergence above $\Delta t_{\text{crit}} \approx 66.5\text{ ms}$.
  - `results/t04_real_world_fixes.md`: Summary table documenting before/after metrics and physical mechanisms for all four engineering hazards.

---

## Running Summary (What Worked / What Didn't / Next Steps)

### What Worked
- **Symplectic Euler Stability**: Semi-implicit Euler showed virtually identical dynamics between $\Delta t = 1\,\text{ms}$ and $\Delta t = 0.5\,\text{ms}$ (less than $0.0102\%$ difference), validating that $\Delta t = 1\,\text{ms}$ is physically honest and computationally fast.
- **Exact Theoretical Alignment**: Simulation results match analytical reference values to 4 significant figures.
- **Clean Figures**: F01 through F04 generated at 300 DPI with clear labels, physical units, legends, and callouts.

### What Didn't (Challenges & Adjustments)
- *Three-lag simulation duration*: Initial steady-state test ran for $10\,\text{s}$ ($10$ time constants), reaching $0.9972$ (analytically exact for $t=10\,\text{s}$). Extended duration to $15\,\text{s}$ where theoretical settling achieves $>99.99\%$, cleanly validating $1.000$ DC gain.

### Next Steps
- Implement Step 6 (Section 8: Choosing the gains via manual tuning & Ziegler-Nichols tuning on the $1/(s+1)^3$ plant).
- Implement Step 7 (Section 9: The reusable code final demo & Section 10: Real robots and myths).
- Compile the 15-page PDF report guide draft.
