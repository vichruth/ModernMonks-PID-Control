"""Master Pipeline Runner (Step 1 & Step 2).

Re-generates all experimental figures and verification checks for Day 1 (Sections 1-4).

Usage:
    python3 run_all.py
"""

import sys
import os
import subprocess
import time


EXPERIMENT_SCRIPTS = [
    ("experiments/s01_loop_diagram.py", "Figure 1: Closed-Loop Architecture & Shower Analogy"),
    ("experiments/s02_pid_terms.py", "Figure 2: PID Term Decomposition"),
    ("experiments/s03_plant.py", "Figure 3: Open-Loop Physics & 20 N Holding Force"),
    ("experiments/s04_metrics.py", "Figure 4: Step Response Performance Criteria"),
]


def run_pipeline():
    print("=" * 65)
    print("      PID CONTROL REPRODUCIBILITY PIPELINE (DAY 1)               ")
    print("=" * 65)

    start_time = time.time()

    # 1. Run Sanity Verification Suite
    print("\n[1/2] Running Physics & Math Assertion Suite (test_sanity.py)...")
    result = subprocess.run([sys.executable, "tests/test_sanity.py"], capture_output=True, text=True)
    if result.returncode != 0:
        print("ERROR: Sanity tests failed!")
        print(result.stdout)
        print(result.stderr)
        sys.exit(1)
    else:
        print("  ✓ All theoretical assertions and reference checks passed.")

    # 2. Execute Experiment Scripts
    print("\n[2/2] Regenerating Step 2 Figures (Sections 1-4)...")
    for script, desc in EXPERIMENT_SCRIPTS:
        if os.path.exists(script):
            print(f"  -> Running {script} ({desc})...")
            res = subprocess.run([sys.executable, script], capture_output=True, text=True)
            if res.returncode != 0:
                print(f"ERROR executing {script}:")
                print(res.stderr)
                sys.exit(1)
            else:
                for line in res.stdout.strip().split("\n"):
                    if line:
                        print(f"     {line}")
        else:
            print(f"  [SKIPPED] {script} (Script not yet created)")

    elapsed = time.time() - start_time
    print("\n" + "=" * 65)
    print(f"  PIPELINE COMPLETE: Step 1 & 2 figures regenerated in {elapsed:.2f}s")
    print("=" * 65)


if __name__ == "__main__":
    run_pipeline()
