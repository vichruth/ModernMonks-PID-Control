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
  - Implemented `pidlab/plants.py` containing:
    - `MassSpringDamper` ($m=1.0\,\text{kg}, c=10.0\,\text{N}\cdot\text{s/m}, k=20.0\,\text{N/m}$) using symplectic / semi-implicit Euler integration.
    - `ThreeLags` ($G(s) = 1/(s+1)^3$) for Ziegler-Nichols tuning experiments.

---

## Running Summary (What Worked / What Didn't / Next Steps)

### What Worked
- Clean mathematical separation: plant dynamics strictly separated from controller and metrics.
- Symplectic Euler chosen for mechanical plant ensures numerical stability without adding external solver bloat.

### What Didn't (Challenges & Adjustments)
- *To be populated as experiments run.*

### Next Steps
- Implement `pidlab/pid.py` (Parallel PID form, clamping anti-windup, derivative on measurement, first-order low-pass filter).
- Implement `pidlab/sim.py` (Discrete closed-loop simulation engine with multi-rate controller support and Gaussian noise).
- Implement `pidlab/metrics.py` (Robust calculation of rise time, overshoot, settling time, steady-state error, and peak force).
- Implement `pidlab/style.py` (Plotting styling and figure export utilities).
- Write and run `tests/test_sanity.py` to assert simulated outputs match reference numbers from `Plan.md` Section 2.
