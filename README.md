# PID Control: Technical Assignment

**Role:** Physical AI Research Intern  
**Company:** Modern Monk Labs  
**Candidate:** Vichruth (hr@modernmonklabs.com)  

---

## 1. Approach & Methodology

This project implements a complete, self-contained study and textbook-grade guide on **PID (Proportional-Integral-Derivative) control** built strictly from first principles.

### Physics-First Modeling (Pure NumPy)
Rather than relying on black-box control libraries (`scipy`, `python-control`, or `simple-pid`), the entire dynamics engine is built from scratch:
* **The Mechanical Plant:** A second-order mass-spring-damper system ($m \ddot{x} + c \dot{x} + k x = u$) with $m = 1.0\text{ kg}$, $c = 10.0\text{ N}\cdot\text{s/m}$, and $k = 20.0\text{ N/m}$.
* **Numerical Integration:** Uses **symplectic (semi-implicit) Euler integration** ($v_{k+1} = v_k + a \cdot \Delta t$, $x_{k+1} = x_k + v_{k+1} \cdot \Delta t$) at $\Delta t = 1\text{ ms}$, ensuring physical energy conservation and numerical stability.
* **Equilibrium Analysis:** Demonstrates why holding the block displaced at $x = 1.0\text{ m}$ against the spring strictly requires a continuous steady-state holding force of $u = k \cdot x = 20\text{ N}$.

### Controller Architecture & Real-World Safeguards
The discrete parallel PID controller ($u = K_p e + K_i \int e \, dt + K_d \frac{de}{dt}$) incorporates four practical engineering fixes:
1. **Clamping Anti-Windup (Conditional Integration):** Halts integrator accumulation when the actuator hits its $30\text{ N}$ force limit and the error pushes further into saturation, preventing massive overshoot.
2. **Derivative on Measurement:** Differentiates the plant measurement ($-K_d \frac{dy}{dt}$) instead of error to eliminate instantaneous $30,000\text{ N}$ setpoint spikes ("kick").
3. **First-Order Low-Pass Filter on D:** Filters high-frequency sensor noise ($\sigma = 1\text{ mm}$) to prevent actuator chatter.
4. **Multi-Rate Zero-Order Hold (ZOH):** Models the physical reality of continuous plant dynamics integrated at $1\text{ ms}$ paired with digital controllers updating at slower intervals ($20\text{ ms} - 100\text{ ms}$).

### AI Usage & Verification Disclosure
In accordance with the assignment guidelines, AI assistance (Gemini / Antigravity) was utilized as an interactive pair-programming partner:
* **What AI helped with:** Generating repository boilerplate, automating Matplotlib plotting routines, and formatting Markdown tables.
* **What was derived and verified independently:** All differential equations, damping ratio derivations ($\zeta = \frac{c + K_d}{2\sqrt{m(k + K_p)}}$), steady-state error formulas ($e_{ss} = \frac{k}{K_p + k}$), Ziegler-Nichols frequency analysis ($K_u = 8, T_u = 3.63\text{ s}$), and physical predictions. Every line of code in `pidlab/` and `tests/` was independently reviewed and verified against theory before inclusion.

---

## 2. Setup Steps

The codebase is lightweight and requires only standard Python 3.10+ with NumPy and Matplotlib.

```bash
# 1. Clone repository
git clone <repo-url>
cd ModernMonks

# 2. Install dependencies (NumPy and Matplotlib only)
pip install -r requirements.txt
```

---

## 3. How to Run the Code

### Option A: One-Command Master Pipeline (`run_all.py`)
To execute the automated verification suite and regenerate all report figures and benchmark tables from scratch:

```bash
python3 run_all.py
```

### Option B: Run Automated Sanity Tests (`tests/test_sanity.py`)
To verify all physics, math formulas, and reference benchmarks against exact analytical theory:

```bash
python3 tests/test_sanity.py
```

*Expected output:*
```text
Running PID Lab Sanity Verification Suite...
  [PASS] test_holding_force_settles_at_1m
  [PASS] test_p_only_steady_state_errors
  [PASS] test_dt_sensitivity (dt=1ms: 4.27%, dt=0.5ms: 4.28%, diff=0.0102%)
  [PASS] test_unbounded_saturation_invariance
  [PASS] test_windup_clamping_effectiveness (None: 17.6%, Clamp: 0.0%)
  [PASS] test_setpoint_kick_suppression (D-on-error: 30300 N, D-on-meas: 300 N)
  [PASS] test_three_lags_steady_state

ALL SANITY TESTS PASSED SUCCESSFULLY! (100% agreement with theory & reference benchmarks)
```

### Option C: Interactive CLI Playground (`playground.py`)
An interactive command-line tool built for testing what-if scenarios and interview predictions:

```bash
# Baseline PID run (Kp=300, Ki=300, Kd=30)
python3 playground.py --kp 300 --ki 300 --kd 30

# What-if: Mass increases to 2 kg with 30 N force limit and no anti-windup
python3 playground.py --kp 300 --ki 300 --kd 30 --mass 2.0 --umax 30 --no-antiwindup

# What-if: Add 1 mm sensor noise with derivative filter
python3 playground.py --kp 300 --ki 300 --kd 30 --noise 0.001 --d-tau 0.02 --save test_noise.png
```

---

## 4. Repository Structure & Generated Deliverables

```
.
├── run_all.py                # Single script to regenerate every figure and table
├── playground.py             # Interactive CLI tool for interview what-if experiments
├── requirements.txt          # Python dependencies (NumPy >= 2.0, Matplotlib >= 3.9)
├── README.md                 # Setup, approach, and execution guide
├── PROJECT-LOG.md            # Living development log (what worked, what didn't, next steps)
├── AI_USAGE.md               # Transparent disclosure of AI assistance
│
├── pidlab/                   # Reusable control & simulation core (no plotting inside)
│   ├── plants.py             # MassSpringDamper & ThreeLags dynamic models
│   ├── pid.py                # Parallel PID with clamping, D-on-measurement, and filtering
│   ├── sim.py                # Closed-loop multi-rate runner with Gaussian noise
│   ├── metrics.py            # Standardized rise, overshoot, settling, and error calculations
│   └── style.py              # Consistent project color palette and 300 DPI exports
│
├── experiments/              # Standalone experiment scripts generating report figures
│   ├── s01_loop_diagram.py   # Figure 1: Control loop architecture & shower analogy
│   ├── s02_pid_terms.py      # Figure 2: PID term decomposition (P, I, D force curves)
│   ├── s03_plant.py          # Figure 3: Open-loop step responses & 20 N holding force
│   ├── s04_metrics.py        # Figure 4: Step response criteria annotations
│   ├── s05_each_part.py      # Figures 5-8 & Tables 1-2: P, I, D parameter sweeps
│   └── ...                   # Sections 6-9 in progress
│
├── tests/
│   └── test_sanity.py        # Numerical assertion checks against analytical theory
├── figures/                  # Generated 300 DPI PNG figures for the report
├── results/                  # Generated benchmark Markdown/CSV tables
└── notes/
    ├── prediction_log.md     # Pre-run predictions vs simulated outcomes
    └── interview_qa.md       # Physics intuition and answers for what-if interview prep
```
