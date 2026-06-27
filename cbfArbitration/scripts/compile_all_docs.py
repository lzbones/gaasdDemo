#!/Users/qingxu/.ai-env/bin/python3
"""
批量编译 doc/<func>/<func>.tex -> .pdf；每份编译两次。
"""
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOC = ROOT / "doc"
FUNCS = [
    "rotateVectorByMinusYaw",
    "computeBarrierSquared",
    "computeBarrierFirstDerivative",
    "computeBarrierSecondDerivativeFree",
    "computeObstacleAffineConstraint",
    "clampControlInput",
    "solveQpHildreth",
    "cbfControlRevision",
]

def compile_one(tex_path: Path):
    out_dir = tex_path.parent
    cmd = ["xelatex", "-interaction=nonstopmode", "-output-directory", str(out_dir), str(tex_path)]
    for run_idx in range(2):
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(f"[FAIL] {tex_path.name} pass {run_idx + 1}")
            print(r.stdout[-3000:])
            return False
    pdf_path = out_dir / (tex_path.stem + ".pdf")
    if not pdf_path.exists():
        print(f"[FAIL] no pdf for {tex_path.name}")
        return False
    for ext in (".aux", ".log", ".toc", ".out"):
        aux = out_dir / (tex_path.stem + ext)
        if aux.exists():
            aux.unlink()
    print(f"[OK]  {pdf_path.relative_to(ROOT)}")
    return True


def main():
    ok = 0
    for name in FUNCS:
        tex = DOC / name / (name + ".tex")
        if not tex.exists():
            print(f"[SKIP] {tex} not found")
            continue
        if compile_one(tex):
            ok += 1
    print(f"\nCompiled {ok}/{len(FUNCS)} PDFs")
    sys.exit(0 if ok == len(FUNCS) else 1)


if __name__ == "__main__":
    main()
