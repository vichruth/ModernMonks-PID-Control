"""Master Pipeline Runner.

Runs the sanity verification suite, regenerates all experimental figures (F01-F17)
and benchmark tables, and compiles report/PID_Control_Guide.pdf when pdflatex is available.

Usage:
    python3 run_all.py
"""

import os
import shutil
import subprocess
import sys
import time


EXPERIMENT_SCRIPTS = [
    ("experiments/s01_loop_diagram.py", "Figure 1: Closed-Loop Architecture & Shower Analogy"),
    ("experiments/s02_pid_terms.py", "Figure 2: PID Term Decomposition"),
    ("experiments/s03_plant.py", "Figure 3: Open-Loop Physics & 20 N Holding Force"),
    ("experiments/s04_metrics.py", "Figure 4: Step Response Performance Criteria"),
    ("experiments/s05_each_part.py", "Figures 5-8 & Tables 1-2: P, I, D Isolation Studies"),
    ("experiments/s06_compare.py", "Figures 9-10 & Table 3: P/PI/PD/PID Architecture Comparison & Gain Selection"),
    ("experiments/s07_real_world.py", "Figures 11-14 & Table 4: Real-World Engineering Hazards & Fixes"),
    ("experiments/s08_tuning.py", "Figures 15-16 & Table 5: Ziegler-Nichols Identification & Tuning on 3-Lag Plant"),
    ("experiments/s09_final_demo.py", "Figure 17 & Table 6: Production Hardened PID Final Verification"),
]


def run_pipeline():
    print("=" * 65)
    print("      PID CONTROL REPRODUCIBILITY PIPELINE (MODERN MONK LABS)    ")
    print("=" * 65)

    start_time = time.time()

    # 1. Run Sanity Verification Suite
    print("\n[1/3] Running Physics & Math Assertion Suite (tests/test_sanity.py)...")
    result = subprocess.run([sys.executable, "tests/test_sanity.py"], capture_output=True, text=True)
    if result.returncode != 0:
        print("ERROR: Sanity tests failed!")
        print(result.stdout)
        print(result.stderr)
        sys.exit(1)
    else:
        print("  [PASS] All theoretical assertions and reference checks passed.")

    # 2. Execute Experiment Scripts
    print("\n[2/3] Regenerating All Figures (F01 - F17) and Results Tables...")
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
            print(f"  [SKIPPED] {script} (Script not found)")

    # 3. Compile Publication PDF Guide (two passes -> report/PID_Control_Guide.pdf)
    tex_path = "report/pid_control_guide.tex"
    tmp_pdf = "report/pid_control_guide.pdf"
    final_pdf = "report/PID_Control_Guide.pdf"

    print(f"\n[3/3] Compiling Publication PDF Guide ({final_pdf})...")
    if not os.path.exists(tex_path):
        print(f"  [SKIPPED] {tex_path} not found; skipping PDF compilation.")
    elif shutil.which("pdflatex") is None:
        print(f"  [SKIPPED] pdflatex not found on PATH; skipping PDF compilation ({final_pdf} unchanged).")
    else:
        compiled_ok = True
        for pass_num in (1, 2):
            res = subprocess.run(
                ["pdflatex", "-interaction=nonstopmode", "-output-directory=report", tex_path],
                capture_output=True,
                text=True,
                errors="replace",
            )
            if res.returncode != 0:
                print(f"  ERROR: pdflatex pass {pass_num} failed with return code {res.returncode}.")
                compiled_ok = False
                break

        if compiled_ok and os.path.exists(tmp_pdf):
            shutil.copy2(tmp_pdf, final_pdf)
            os.remove(tmp_pdf)
            print(f"  [PASS] PDF compiled (2 passes) -> {final_pdf} (removed {tmp_pdf}).")
        elif not compiled_ok:
            print(f"  [WARN] PDF compilation failed; keeping existing {final_pdf} if present.")

    elapsed = time.time() - start_time
    print("\n" + "=" * 65)
    print(f"  PIPELINE COMPLETE: Figures, tables, and checks finished in {elapsed:.2f}s")
    print("=" * 65)


if __name__ == "__main__":
    run_pipeline()
