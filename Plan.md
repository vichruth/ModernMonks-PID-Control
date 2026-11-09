# PLAN.md: PID Control Intern Assignment (Modern Monk Labs)

How to use this file: read it top to bottom once, then follow **section 8 (schedule)** and **section 6 (build recipe)** in order.
General coding behaviour (keep it simple, change only what's needed, verify before claiming done) comes from your `AGENTS.md`. This file only holds the project-specific facts, decisions and numbers.

---

## 0. Snapshot

| Item | Detail |
|---|---|
| Deadline | **Fri 13 Nov 2026, 11:59 PM IST**. Aim to submit **Thu 12 Nov night**; Fri is buffer only. |
| Submit by | Replying to the HR email thread (hr@modernmonklabs.com) |
| What to send | (1) PDF guide, **max 15 pages**; (2) GitHub link (public, or private shared with hr@modernmonklabs.com); (3) README in the repo; (4) a **short write-up** (what worked / what didn't / what next). The HR email asks for the write-up and the assignment PDF doesn't mention it, so send both to be safe. |
| Code rules | **NumPy + Matplotlib only.** You write the PID and the simulator yourself. No `scipy`, `python-control`, `simple-pid`, etc. (Don't use `scipy.signal.find_peaks` either; write a 5-line peak finder.) |
| AI rule | Allowed, **but mention briefly how you used it** (put it in the PDF and in `AI_USAGE.md`). |
| Interview | 30 min. They change a gain / the mass / the noise / the force limit and you **predict before running**, then explain. |
| Scoring | Concepts 25 · Calculations & code 20 · Easy to understand 20 · Graphs & writing 15 · **Interview 20** |

---

## 1. What the project actually is

You are writing **a short beginner's textbook chapter on PID control**, and *proving every claim with graphs from code you wrote*.
The graders want evidence that you (a) understand PID, (b) can explain it simply, (c) back claims with numbers, (d) know what goes wrong in real machines, and (e) can reason about it live.

The pipeline is: **simulate → measure → compare with theory → explain → write.**

The machine you control is a 1 kg block on a spring and damper: `m·x'' + c·x' + k·x = u`, with m = 1 kg, c = 10 N·s/m, k = 20 N/m. Goal: move it to **1 m** and hold it there. `u` is the force the controller pushes with.

---

## 2. Reference numbers (check YOUR code against these)

I ran a quick reference simulation (parallel-form PID, D on measurement, dt = 1 ms, semi-implicit Euler, setpoint 1 m, start at rest). **Do not paste these into your PDF.** Your own code must produce the PDF numbers. Use these to catch bugs: if yours differ by more than ~1-2%, look for a bug or a different metric definition.

**Plant facts**
- Poles of the open-loop plant: s = −2.76 and −7.24. ζ = 1.118, so it is **overdamped** (no bouncing on its own).
- Holding force at 1 m: `u = k·x = 20 N` (at rest the damper does nothing, so only the spring matters).

**P-only (steady-state error = k/(Kp+k), x_ss = Kp/(Kp+k))**

| Kp | theory error | simulated error | overshoot (vs setpoint) |
|---|---|---|---|
| 10 | 0.6667 | 0.6667 | 0% |
| 50 | 0.2857 | 0.2857 | 0% |
| 100 | 0.1667 | 0.1667 | 0% |
| 300 | 0.0625 | 0.0625 | 31.2% |
| 1000 | 0.0196 | 0.0196 | 57.6% |

**Damping theory (D on measurement makes P/PD an exact 2nd-order system):**
closed loop `m·s² + (c+Kd)·s + (k+Kp)`, so `ζ = (c+Kd) / (2·√(m·(k+Kp)))`, overshoot **relative to the final value** = `exp(−πζ/√(1−ζ²))`.
- Kp = 300, Kd = 0: ζ = 0.28, predicted peak = 0.9375 × 1.4007 = 1.313 → **31.3% over setpoint**; simulated 31.2% ✔
- Kp = 1000, Kd = 0: ζ = 0.157, predicted 57.6%; simulated 57.56% ✔
- Kp = 300, Kd = 10: ζ = 0.559 → 12.0% vs final value → **5.0% over the 1 m setpoint** (final value is 0.9375 m); simulated 4.85% ✔. Watch the definition: reporting 11-12% here while Kd = 0 is reported as 31.2% (vs setpoint) mixes two definitions.
- Kp = 300, Kd = 30: ζ = 1.118 → overdamped → **0% overshoot**; simulated 0% ✔
- Run every response long enough to truly settle (≥ 10-15 s). At a 3-4 s horizon the PID 300/300/30 "final error" still reads about −0.004 m; at 15 s it is 0.

**Combinations (unlimited force)**

| Controller | Overshoot | Settling (2%) | Rise (10-90%) | Final error |
|---|---|---|---|---|
| PI 100 / 200 | 21.9% | 0.84 s | 0.148 s | 0 |
| PD 300 / Kd 30 | 0% | never (stays 6.25% short) | 0.305 s | 0.0625 |
| PID 300 / 300 / 30 | 4.27% | 1.42 s | 0.206 s | 0 |
| PID 600 / 300 / 30 | 1.18% | 0.155 s | 0.102 s | 0 (but needs 600 N at t = 0) |

**Real-world problems (PID 300/300/30 unless stated)**

| Problem | Before fix | After fix |
|---|---|---|
| (a) Windup, limit 30 N | overshoot 17.6%, settle 2.98 s | clamping anti-windup: 0% overshoot, settle 1.06 s |
| (b) Noise σ = 1 mm, std of u | raw D: **43 N** (theory `Kd·√2·σ/dt` = 42.4 N) | filtered τ = 0.02 s: 1.76 N; τ = 0.05 s: 0.90 N; PI without D: 0.30 N |
| (c) Setpoint kick, first sample | D on error: **30,300 N** | D on measurement: 300 N |
| (d) Slow controller update | stable and near-identical up to ~40 ms; 50 ms → 11.9% overshoot; **100 ms → diverges** | find your own threshold by bisection |

**Interview-style what-ifs (PID 300/300/30)**

| Change | Result |
|---|---|
| mass 2 kg | overshoot 6.9%, settle 1.35 s |
| mass 0.5 kg | overshoot 4.0%, settle 1.45 s |
| Kp = 100 / 200 / 600 | overshoot 24.8% / 9.8% / 1.2%; settle 2.6 / 1.8 / 0.16 s |
| force limit 22 / 25 / 40 / 60 N (no anti-windup) | overshoot 10.0% / 19.6% / 12.7% / 8.5% (**not monotonic**: don't guess, run it) |

**Ziegler-Nichols on 1/(s+1)³** (Ku = 8 exactly from theory: phase = −180° at ω = √3, |G| = 1/8; Tu = 2π/√3 = 3.628 s; my sweep measured 3.625 s)

| Rule | Kp / Ki / Kd | Overshoot | Settling |
|---|---|---|---|
| ZN classic PID (0.6·Ku, Tu/2, Tu/8) | 4.80 / 2.65 / 2.18 | **52.4%** | 9.9 s |
| ZN classic PI | 3.60 / 1.19 / 0 | 56.0% | 30.7 s |
| ZN classic P | 4.00 / 0 / 0 | 23.4% | never (error 1/(1+4) = 0.2) |
| "some overshoot" | 2.64 / 1.46 / 3.19 | 24.9% | 12.8 s |
| "no overshoot" | 1.60 / 0.88 / 1.93 | 20.9% (label is only a rule of thumb) | 14.0 s |
| Tyreus-Luyben | 3.64 / 0.46 / 2.09 | 0.94% | 16.9 s (slow tail) |

Your "improve it" target: **overshoot under ~10% with settling under ~10 s**. Classic ZN is fast but wild; Tyreus-Luyben is calm but slow; the sweet spot is between.

---

## 3. Decisions and alternatives

Pick the **bold** option unless you have a reason not to. Write your choice and reason in the README (one line each); this is exactly what an interviewer may ask.

| Decision | Options | Why / trade-off |
|---|---|---|
| Simulator method | **Semi-implicit Euler, dt = 1 ms** · RK4 · exact discretisation | Euler is simple to explain. I checked halving dt (1 ms → 0.5 ms) changes overshoot 4.27% → 4.28%, so it's fine. Optionally cross-check one run with RK4 (shows care). |
| PID form | **Parallel: u = Kp·e + Ki·∫e + Kd·de/dt** · Standard: Kp(1 + 1/(Ti·s) + Td·s) | Parallel is the one the assignment writes (Kp, Ki, Kd). Convert for Ziegler-Nichols: Ki = Kp/Ti, Kd = Kp·Td. Show the conversion. |
| Derivative input | **On measurement (−dy/dt)** · on error | Same behaviour for a constant setpoint, but no kick on setpoint changes (30,300 N vs 300 N). Show **both** in 7c. |
| Derivative filter | **First-order low-pass: d_f += α·(d − d_f), α = dt/(τ+dt)** · moving average | One line of code, one parameter. Cost: adds lag; mention it. |
| Anti-windup | **Clamping (stop integrating while saturated and pushing further)** · back-calculation · integrator limit | Clamping is easiest to explain. Optionally compare with back-calculation as a stretch goal. |
| Noise model for "±1 mm" | **Gaussian σ = 1 mm, seed 42** · uniform ±1 mm | The task is ambiguous; **say which you chose in the PDF**. (Uniform ±1 mm has σ = 0.58 mm.) |
| Slow-update test | **Plant stepped at 1 ms; controller updates every N steps (zero-order hold)** · simulate at the slow dt | Keeping the plant fine and the controller slow is the physically honest model. |
| Rise time | **10→90% of the setpoint** · of the final value | For weak P (Kp = 10, 50) the block never reaches 90% of 1 m. Report **"n/a (never reaches 90%)"**, don't fake a number. |
| Overshoot | **(peak − setpoint)/setpoint** · (peak − final)/final | The assignment says "going past the target", so use setpoint. The theory formula gives overshoot vs the *final* value, so convert (see section 2) and say so. |
| Settling time | **last time |y − 1| > 2% of 1 m** | For P-only the error is above 2% forever, so report **"never settles"**. |
| Steady-state error | **1 − mean(y over last 5% of samples)** | Run long enough (≥ 10-15 s) that it has really settled. |
| Gain tuning for the final answer | **Hand-tuned with a stated rule, plus a small grid search** · hand-tuned only | Grid search is ~20 lines of NumPy. State the score you minimised (e.g. overshoot < 5%, settling < 1.5 s, peak force reported). |
| Document tool | **Google Docs or Word, export PDF** (easiest, page count obvious) · Overleaf/LaTeX (best equations, floats can jump and blow the page limit) · Markdown + pandoc (fiddly setup) | If you already know LaTeX use Overleaf. Otherwise Docs/Word; both have an equation editor. |
| Figures in the doc | **PNG, 300 dpi, width 6.5 in** · SVG/PDF vector | PNG is easiest to paste. Keep each under ~300 KB. |
| Code layout | **Small package (`pidlab/`) + one script per section** · single notebook · one huge script | Packages let `run_all.py` and `playground.py` reuse the same PID. Notebook is optional extra, not the main deliverable. |

---

## 4. Exact folder structure

```
pid-control-assignment/
├── README.md                 # setup, how to run, which script makes which figure, decisions, known gaps
├── WRITEUP.md                # 1 page: approach, what worked, what didn't, what next
├── AI_USAGE.md               # 5-10 lines: how AI was used (and what you verified yourself)
├── PLAN.md                   # this file
├── AGENTS.md                 # your general coding guidelines
├── requirements.txt          # numpy, matplotlib (pin the versions you used)
├── .gitignore                # __pycache__/, .venv/, .DS_Store
├── run_all.py                # regenerates EVERY figure + table from scratch
├── playground.py             # interview tool: change a gain/mass/noise/limit from the command line
│
├── pidlab/                   # the reusable code (no plotting in here)
│   ├── __init__.py
│   ├── plants.py             # MassSpringDamper(m, c, k), ThreeLags()  -> .step(u, dt), .output
│   ├── pid.py                # PID class: P, I, D, saturation, anti-windup, D filter, D on measurement
│   ├── sim.py                # closed-loop runner (plant dt vs controller dt, noise, setpoint, logging)
│   ├── metrics.py            # rise, overshoot, settling, steady-state error (one definition, used everywhere)
│   └── style.py              # colours, font sizes, figure size, save_fig()
│
├── experiments/              # one script per assignment section; each saves figures + a results table
│   ├── s01_loop_diagram.py
│   ├── s02_pid_terms.py
│   ├── s03_plant.py
│   ├── s04_metrics.py
│   ├── s05_each_part.py
│   ├── s06_compare.py
│   ├── s07_real_world.py
│   ├── s08_tuning.py
│   └── s09_final_demo.py
│
├── tests/
│   └── test_sanity.py        # plain asserts (run: python tests/test_sanity.py)
│
├── figures/                  # generated PNGs (commit them so reviewers see results without running)
├── results/                  # generated tables as .md/.csv (copy into the PDF)
├── notes/
│   ├── prediction_log.md     # predict -> run -> compare -> explain (interview gold)
│   └── interview_qa.md       # your own answers to the what-if questions
└── report/
    ├── PID_Control_Guide.pdf # final submission PDF
    └── source/               # Docs/Word export or .tex, plus any extras
```

**Design rules**
- `pidlab/` never plots. `experiments/` never re-implements PID. One source of truth.
- Every experiment is a `main()` that saves `figures/figNN_*.png` and `results/*.md`. `run_all.py` just calls them in order.
- Use `matplotlib.use("Agg")` so it runs on any machine without a display.
- Noise uses `np.random.default_rng(42)`. Same graphs every time.

**Minimal API (keep it this small)**

```python
class PID:
    def __init__(self, kp, ki, kd, dt, u_min=-np.inf, u_max=np.inf,
                 anti_windup="clamp",      # "none" | "clamp" | "backcalc"
                 d_on="measurement",       # "measurement" | "error"
                 d_filter_tau=0.0):        # seconds; 0 = no filter
    def reset(self): ...
    def update(self, setpoint, measurement) -> float: ...   # returns force u in N
    # also expose last P, I, D terms for plotting

class MassSpringDamper:
    def __init__(self, m=1.0, c=10.0, k=20.0): ...
    def step(self, u, dt): ...                # semi-implicit Euler
    @property
    def x(self): ...

def simulate(plant, controller, setpoint, T, dt_sim=0.001, dt_ctrl=None,
             noise_sigma=0.0, seed=42) -> dict   # t, y, y_meas, u, p, i, d, r
```

`playground.py` example (build this early; it doubles as interview practice):
```
python playground.py --kp 300 --ki 300 --kd 30 --mass 2 --noise 0.001 --umax 30 --no-antiwindup
```
It prints the four metrics and shows the plot.

---

## 5. Figures and tables to generate

Use **multi-panel figures** to stay inside 15 pages. Consistent colours everywhere: **P = blue, I = green, D = red, setpoint = black dashed, total/output = dark grey**. Every figure: title, axis labels with units, legend, and a one-line caption in the PDF saying what it shows.

| ID | Section | Shows | Script |
|---|---|---|---|
| F01 | 1 | Loop diagram: target → controller → machine → sensor → back (drawn with matplotlib boxes and arrows) | s01 |
| F02 | 2 | One sample error curve and the P, I, D terms it produces (3 small panels) | s02 |
| F03 | 3 | Plant open loop: 20 N push → 1 m, 10 N → 0.5 m, 0 N → 0 m (why 20 N is needed) | s03 |
| F04 | 4 | One step response with **rise, overshoot, settling, steady-state error all marked** | s04 |
| F05 | 5 | P only, Kp = 10, 50, 100, 300, 1000 | s05 |
| F06 | 5 | Steady-state error: **theory curve k/(Kp+k) with simulated dots on top**; plus ζ/overshoot theory vs sim | s05 |
| F07 | 5 | Add I: Kp = 100, Ki = 0, 50, 200, 500 | s05 |
| F08 | 5 | Add D: Kp = 300, Kd = 0, 10, 30, 60 (bouncing shrinks) | s05 |
| F09 | 6 | P vs PI vs PD vs PID on one graph, with force panel underneath | s06 |
| F10 | 6 | Final gains: response and P/I/D contributions over time | s06 |
| F11 | 7a | Windup: position, force and integrator state, before vs after anti-windup | s07 |
| F12 | 7b | Noise: control force with raw D vs filtered D | s07 |
| F13 | 7c | Setpoint kick: first ~50 ms of force, D on error vs on measurement | s07 |
| F14 | 7d | Slow update: responses for 1, 20, 50, 100 ms | s07 |
| F15 | 8 | Finding Ku: P-only at Kp below / at / above Ku, with Tu marked | s08 |
| F16 | 8 | ZN classic vs Tyreus-Luyben vs your improved gains | s08 |
| F17 | 9 | Final demo: reusable PID on the test system (with printed metrics) | s09 |

Tables (saved to `results/`): T1 metrics for P gains · T2 theory vs sim error · T3 P/PI/PD/PID comparison (add a **peak force** column) · T4 before/after for each real-world problem · T5 ZN table and results.

---

## 6. Build recipe (do these in order)

For every step: **code → check against theory/section 2 → write the prediction log → only then make the figure.**

### Step 1: core code (Day 1)
1. `plants.py`: MassSpringDamper. Test: constant u = 20 N must settle at x = 1.000 m.
2. `pid.py`: start with plain P, I, D (no extras). Add the extras later, each behind a switch.
3. `sim.py`: closed-loop runner returning arrays.
4. `metrics.py`: four measures with the definitions in section 3. Handle "never reaches 90%" and "never settles".
5. `tests/test_sanity.py` asserts:
   - holding force 20 N gives x = 1 m
   - P-only error matches `k/(Kp+k)` within 1%
   - halving dt changes overshoot by < 0.1 percentage points
   - with u_max = ∞ the saturation code changes nothing

### Step 2: Sections 1-4 (Day 1)
- **S1**: shower analogy in plain words (too cold → turn hot tap → feel it → adjust). Draw F01.
- **S2**: formula `u(t) = Kp·e(t) + Ki·∫e dt + Kd·de/dt`; units: e [m], u [N], Kp [N/m], Ki [N/(m·s)], Kd [N·s/m]. Discrete version: `e[k] = r − y[k]`; `I += e[k]·dt`; `D = (e[k] − e[k−1])/dt`; `u[k] = Kp·e[k] + Ki·I + Kd·D`. Plain-words meaning: P = present, I = past (accumulated), D = trend (how fast the error is changing). F02.
- **S3**: write `m·x'' + c·x' + k·x = u`. At rest, x' = x'' = 0, so `u = k·x`: 20 N to hold 1 m. F03. Mention overdamped (ζ = 1.118).
- **S4**: define four measures, mark them on F04.

### Step 3: Section 5, each part alone (Day 2)
- **P only** (Kp = 10, 50, 100, 300, 1000): closed loop `m·x'' + c·x' + (k+Kp)·x = Kp·r`; steady state `x = Kp/(k+Kp)`, so error = `k/(k+Kp)`. Table T2 of theory vs sim (agreement to 4 decimals in my check). Explain in words: the controller only pushes when there is error, but holding 1 m needs force against the spring. At rest, `Kp·(1 − x) = k·x`, so `x = Kp/(Kp+k)` and the error is `k/(Kp+k)`. (It is **not** `20/Kp`: the force the controller supplies is `Kp·e`, which is less than 20 N unless Kp is huge.)
- **Add I**: at steady state the integral stops changing only when e = 0, and then `Ki·∫e` alone supplies the 20 N. This is the key insight: **I remembers the force needed**. Show it by plotting the I term settling at 20 N.
- **Add D**: `ζ = (c+Kd)/(2·√(m·(k+Kp)))`. Predict overshoot for Kd = 0, 10, 30, 60 *before* running, then compare. For Kp = 300 overshoot vanishes once ζ ≥ 1, which needs Kd ≥ about 25.8 N·s/m. (Check this arithmetic yourself: ζ = 1 when c + Kd = 2·√320.)
- Remember D on measurement keeps the PD closed loop exactly 2nd order, so the formula is exact; with D on error there's an extra zero and it's only approximate. Say so.

### Step 4: Section 6, compare and choose (Day 2)
- Same plot: P, PI, PD, PID. Table with the four measures **and peak force**.
- **Choose final gains with written criteria**, e.g. overshoot < 5%, settling < 1.5 s, zero final error. Candidate to start from: Kp = 300, Ki = 300, Kd = 30 (4.27%, 1.42 s). Run a small grid search around it to justify.
- Honest tension to discuss in the PDF: Kp = 600 is much faster (0.155 s) but needs 600 N at t = 0, impossible with a 30 N motor. That bridges into Section 7.

### Step 5: Section 7, real-world problems (Day 3)
Each one is: **show the problem with a graph, explain why, fix, show the fixed graph, give before/after numbers.**
- **(a) Windup**: limit |u| ≤ 30 N. Plot position, force, and integrator state. Fix: clamping. Expected: overshoot 17.6% → 0%, settling 2.98 → 1.06 s.
- **(b) Noise**: σ = 1 mm. A 1 mm error between two samples 1 ms apart looks like 1 m/s of velocity, times Kd = 30. Expected std(u) ≈ `Kd·√2·σ/dt` ≈ 42 N, nearly the whole 30 N limit. Fix: low-pass filter τ = 0.02-0.05 s (1.76 / 0.90 N). Mention the cost (lag) and show both.
- **(c) Kick**: step of 1 m in one sample with D on error gives `Kd·Δe/dt = 30·1/0.001 = 30,000 N`. Fix: D on measurement (300 N).
- **(d) Slow updates**: plant at 1 ms, controller every 1, 5, 10, 20, 50, 100 ms. Find the instability threshold by bisection. Explain: the controller reacts to old information and corrections arrive late, so it overcorrects.

### Step 6: Section 8, tuning (Day 4)
- **Manual tuning recipe** (write it as numbered steps): (1) set Ki = Kd = 0; (2) raise Kp until it's fast but starts to overshoot; (3) add Kd until the bounce is damped; (4) add Ki until steady error is gone; (5) if overshoot returns, lower Ki or raise Kd; (6) re-check with noise and force limit. Add a small table: raising each gain → rise / overshoot / settling / steady error / noise sensitivity.
- **Ziegler-Nichols on 1/(s+1)³**: simulate as 3 cascaded lags. Sweep Kp with P only; find Ku (the gain where oscillation neither grows nor decays; expect ~8) and Tu (peak-to-peak spacing; expect ~3.63 s). Apply table: P: 0.5·Ku; PI: 0.45·Ku, Ti = Tu/1.2; PID: 0.6·Ku, Ti = Tu/2, Td = Tu/8. Convert to Ki, Kd. Show the 52% overshoot, then improve (e.g. lower Kp, change Ti/Td, or try Tyreus-Luyben) and report before/after numbers.
- Sanity from theory: the 3-lag plant has phase −180° at ω = √3 with |G| = 1/8, so Ku = 8 and Tu = 2π/√3. Quote this as a cross-check.

### Step 7: Sections 9-10 (Day 5)
- **S9**: show the final `PID` class (trimmed) with docstrings, one demo run on the test system, and printed output (the metrics and maybe the first few `u` values).
- **S10**: 4+ real examples, with *what is the error / what is the actuator / what gets tuned*:
  1. Cruise control / motor speed: error = target speed − measured speed, actuator = throttle or voltage.
  2. Drone attitude and altitude: one PID per axis, often cascaded (angle loop outside, rate loop inside).
  3. Robot arm joint: error = target angle − encoder angle, actuator = motor torque; gravity needs I or feed-forward.
  4. Line follower: error = line offset from sensor array, output steers the wheel speed difference.
  5. (extra) Oven / 3D-printer hotend temperature; self-balancing robot.
  **Myths** (pick 4-6): "more gain is always better" (F05: 57.6% overshoot at Kp = 1000); "I fixes everything" (windup, F11); "D predicts the future" (it reacts to the *current rate of change*, and amplifies noise, F12); "PID is old/obsolete" (still the default building block in many machines; if you quote a percentage, cite a real source or leave it out); "one set of gains works everywhere" (mass 0.5 kg vs 2 kg changes the response); "tuning rules like Ziegler-Nichols give the final answer" (52% overshoot here). Tie each myth back to one of *your* figures.
- README, AI_USAGE.md, WRITEUP.md. Run a fresh-clone test.

---

## 7. Writing the PDF

**Pattern for every section:** hook or analogy → plain words → formula → figure → numbers → one-line takeaway.
**Caption template:** *"Figure N. [What it shows]. [Key number]."* e.g. "Figure 8. Adding D (Kp = 300). Overshoot drops from 31.2% to 0% as Kd goes from 0 to 30."

**Page budget (15 max)**

| Part | Pages |
|---|---|
| Title, one-paragraph summary, AI note | 0.5 |
| 1 The basic idea | 1.0 |
| 2 The PID formula | 1.25 |
| 3 The test system | 0.75 |
| 4 How to judge a result | 0.75 |
| 5 Each part alone | 2.75 |
| 6 All together | 1.25 |
| 7 Real-world problems | 2.75 |
| 8 Choosing the gains | 1.5 |
| 9 The code | 1.0 |
| 10 Robots and myths | 1.0 |
| Buffer | 0.5 |

Include a short **"What is missing"** note if anything is unfinished (the assignment explicitly allows this).
Write in your own voice, plain and direct. Avoid generic filler ("it is important to note"); the graders read many of these.

---

## 8. Schedule (today is Fri 6 Nov)

| When | Goal | Done when |
|---|---|---|
| **Fri 6 (tonight)** | Create repo + venv + folder skeleton, `git init`, first commit. Read the assignment PDF once more. Decide document tool. | `pip install -r requirements.txt` works |
| **Sat 7 (Day 1)** | Step 1 + Step 2 (core code, Sections 1-4 figures and draft text) | tests pass; F01-F04 exist |
| **Sun 8 (Day 2)** | Step 3 + Step 4 (Sections 5-6) | theory-vs-sim table matches; final gains chosen with criteria |
| **Mon 9 (Day 3)** | Step 5 (Section 7: windup, noise, kick, slow dt) | before/after numbers for all four |
| **Tue 10 (Day 4)** | Step 6 (Section 8) + `playground.py` | Ku ≈ 8, Tu ≈ 3.63 found by your own sweep |
| **Wed 11 (Day 5)** | Step 7 (Sections 9-10), `run_all.py`, README, AI_USAGE, **first full PDF draft** | PDF assembled, page count known |
| **Thu 12 (Day 6)** | Trim to ≤ 15 pages, proofread, fresh-clone test, WRITEUP.md, prediction practice, **submit tonight** | email sent |
| **Fri 13 (Day 7)** | Buffer only. If you already submitted, practise the interview. Never submit after 6 PM. | |

If you fall behind, cut in this order: stretch goals → back-calculation → grid search → extra myths. **Never cut**: P-only theory check, the four real-world fixes, Ziegler-Nichols, README, AI note.

---

## 9. How to stand out

1. **Theory next to simulation, everywhere.** Predicted vs measured tables. This is the "calculations" and "concepts" marks and it is the exact skill the interview tests.
2. **A prediction log** (`notes/prediction_log.md`): before each run, write one line "I expect …"; after, "got … because …". If you were wrong, say why. It feeds the "what didn't work" write-up.
3. **`playground.py`** so you can re-run any what-if in seconds during interview prep.
4. **Peak force column** in the comparison table. It connects Section 6 to Section 7 and shows engineering judgement.
5. **State definitions and assumptions**, like the noise model, overshoot reference and "never settles". Small, but shows rigour.
6. **Honest limitations**: simulation not hardware, linear plant, Euler method, one noise model.
7. **Consistent visual language** (the colour code in section 5) and numbers in captions.
8. **Stretch goals** (only after everything else is done, in order of value): mass robustness sweep (0.5-2 kg) as one figure → back-calculation vs clamping → grid search over gains with ITAE → relay auto-tune.

**Using AI without hurting yourself:** let the agent write boilerplate and plotting, but you write the predictions, pick the gains, and read every line of `pid.py`, `sim.py` and `metrics.py` until you could re-type them. If you can't explain a line, ask the agent to explain it, then simplify it.

---

## 10. Interview prep

Format: they change something, you **predict first**, then run, then explain. Practice in this order: *say the direction and rough size → say the reason (one equation or physical sentence) → run → reconcile.*

| Change | Think about | My reference (cover this until you've predicted) |
|---|---|---|
| Kp 300 → 100 | `e_ss` (with I it's 0), ζ, overshoot | overshoot 24.8%, settle 2.6 s |
| Kp 300 → 600 | faster, but force at t = 0 | overshoot 1.2%, settle 0.16 s; needs 600 N |
| Mass 1 → 2 kg | slower dynamics, ζ drops | overshoot 6.9%, settle 1.35 s |
| Noise σ 1 mm → 2 mm | which term amplifies it? | D noise doubles (~86 N rms raw) |
| Force limit 30 → 22 N | only 2 N above the 20 N holding force | run it; result is not monotonic |
| Remove D (Kd 30 → 0, Kp = 300, Ki = 300) | ζ drops, bounce returns | **run it yourself**: I did not reference-check this row |
| Remove anti-windup | integrator charges during saturation | 17.6% overshoot, 2.98 s |
| Controller 1 ms → 100 ms | stale information | diverges |
| D on error instead of measurement | step in setpoint | 30,300 N spike |

Also be ready to explain, out loud and without notes:
- Why P alone always leaves an error here (and by how much).
- Why I removes it and what number the integral settles at (20 N).
- What D does physically and why it hates noise.
- What windup is.
- How you found Ku and Tu, and why ZN overshoots.
- Why you chose each design decision in section 3.

---

## 11. Traps checklist (read before submitting)

- [ ] No `scipy`, no control libraries, no ready-made PID anywhere (`grep -rn "import" . | sort -u`).
- [ ] `python run_all.py` works on a fresh clone in a new venv, and recreates **every** graph.
- [ ] Every figure: title, axis labels with units, legend, one-line caption.
- [ ] No vague words in the PDF ("much better"); use numbers.
- [ ] Rise time and settling time shown as "n/a / never" where they truly are.
- [ ] Overshoot reference stated (vs setpoint) and used consistently.
- [ ] Noise model stated (Gaussian σ = 1 mm, seed 42).
- [ ] Plant integrated at fine dt; controller dt separate in the slow-update test.
- [ ] Windup test has the 30 N limit in the PID **and** anti-windup toggled off for "before".
- [ ] PDF ≤ 15 pages; all 10 section headings present, in order.
- [ ] README has: setup, one command to run everything, script → figure map, decisions, known gaps.
- [ ] AI use mentioned in the PDF and `AI_USAGE.md`.
- [ ] Repo is public or shared with hr@modernmonklabs.com; link opens in a private window.
- [ ] Email sent **as a reply** on the original thread, with repo link, PDF attached, and the short write-up.

---

## 12. Submission email (template)

> Subject: Re: Physical AI Research Intern – Next Round: Technical Assignment
>
> Dear Siva Mounika,
>
> Please find my submission for the PID Control assignment:
> - GitHub repository: <link> (README explains how to recreate every graph)
> - PDF guide: attached (N pages)
> - Short write-up: attached / below
>
> I used AI tools for <one line>; details are in the PDF and AI_USAGE.md in the repo.
>
> Thank you, and I look forward to the technical discussion.
>
> Best regards,
> Vichruth