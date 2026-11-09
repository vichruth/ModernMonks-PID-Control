import subprocess
import os
import sys

def build_pdf():
    tex_path = "report/pid_control_guide.tex"
    print(f"Compiling {tex_path} with pdflatex...")
    
    for pass_num in [1, 2]:
        res = subprocess.run(
            ["pdflatex", "-interaction=nonstopmode", "-output-directory=report", tex_path],
            capture_output=True,
            text=True,
            errors="replace"
        )
        if res.returncode != 0:
            print(f"Pass {pass_num} FAILED with returncode {res.returncode}!")
            lines = res.stdout.splitlines()
            for line in lines[-50:]:
                print(line)
            return False
        else:
            print(f"Pass {pass_num} completed successfully.")
            
    pdf_path = "report/pid_control_guide.pdf"
    if os.path.exists(pdf_path):
        res = subprocess.run(["pdfinfo", pdf_path], capture_output=True, text=True, errors="replace")
        for line in res.stdout.splitlines():
            if "Pages:" in line:
                print(f"--> Result: {pdf_path} generated successfully! {line.strip()}")
        return True
    return False

if __name__ == "__main__":
    build_pdf()
