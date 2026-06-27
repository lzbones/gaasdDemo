#!/Users/qingxu/.ai-env/bin/python3
import os
import subprocess
from pathlib import Path

FUNCTIONS = [
    {"name": "add", "Name": "Add"},
    {"name": "sub", "Name": "Sub"},
    {"name": "mul", "Name": "Mul"},
    {"name": "div", "Name": "Div"},
    {"name": "abs", "Name": "Abs"},
    {"name": "sign", "Name": "Sign"},
    {"name": "clamp", "Name": "Clamp"},
    {"name": "saturate", "Name": "Saturate"},
    {"name": "sqrt", "Name": "Sqrt"},
    {"name": "reciprocalSqrt", "Name": "ReciprocalSqrt"},
    {"name": "pow", "Name": "Pow"},
    {"name": "exp", "Name": "Exp"},
    {"name": "log", "Name": "Log"},
    {"name": "mod", "Name": "Mod"},
    {"name": "ceil", "Name": "Ceil"},
    {"name": "floor", "Name": "Floor"},
    {"name": "round", "Name": "Round"},
    {"name": "min", "Name": "Min"},
    {"name": "max", "Name": "Max"},
    {"name": "bias", "Name": "Bias"}
]

ROOT = Path(__file__).parent.parent.parent
PYTHON = "/Users/qingxu/.ai-env/bin/python3"

def run_cmd(args, cwd=None):
    res = subprocess.run(args, capture_output=True, text=True, cwd=cwd)
    return res.returncode, res.stdout, res.stderr

def main():
    print("=== Starting Auto Compilation and Verification Pipeline ===")
    
    summary = []
    
    for func in FUNCTIONS:
        name = func["name"]
        Name = func["Name"]
        print(f"\n--------------------------------------------------")
        print(f"Processing Module: {name} (MBD: {Name})")
        print(f"--------------------------------------------------")
        
        module_dir = ROOT / name
        build_dir = module_dir / "build"
        build_dir.mkdir(exist_ok=True)
        
        # 1. CMake configure
        print(f"[{name}] Configuring CMake...")
        code, out, err = run_cmd(["cmake", "-S", ".", "-B", "build"], cwd=module_dir)
        if code != 0:
            print(f"[{name}] CMake Config Failed:\n{err}")
            summary.append({"name": name, "step": "CMake Config", "status": "FAIL", "msg": err})
            continue
            
        # 2. CMake build
        print(f"[{name}] Building...")
        code, out, err = run_cmd(["cmake", "--build", "build"], cwd=module_dir)
        if code != 0:
            print(f"[{name}] Build Failed:\n{err}")
            summary.append({"name": name, "step": "Build", "status": "FAIL", "msg": err})
            continue
            
        # 3. CTest (C++ & MBD unit tests)
        print(f"[{name}] Running Tests...")
        code, out, err = run_cmd(["ctest", "--output-on-failure"], cwd=build_dir)
        if code != 0:
            print(f"[{name}] Tests Failed:\n{out}\n{err}")
            summary.append({"name": name, "step": "Test", "status": "FAIL", "msg": out + err})
            continue
        else:
            print(f"[{name}] Tests Passed!")
            
        # 4. Generate C++ Plot
        print(f"[{name}] Generating C++ Plot...")
        plot_cpp_script = module_dir / "tests" / "cppTest" / "output" / f"plot_{name}.py"
        json_cpp_cases = module_dir / "tests" / "cppTest" / "unit" / f"{name}_cases.json"
        code, out, err = run_cmd([PYTHON, str(plot_cpp_script), str(json_cpp_cases)], cwd=module_dir)
        if code != 0:
            print(f"[{name}] C++ Plot generation failed:\n{err}")
            summary.append({"name": name, "step": "C++ Plot", "status": "FAIL", "msg": err})
            continue
            
        # 5. Generate MBD Plot
        print(f"[{name}] Generating MBD Plot...")
        plot_mbd_script = module_dir / "tests" / "mbdTest" / "output" / f"plot_{Name}.py"
        json_mbd_cases = module_dir / "tests" / "mbdTest" / "unit" / f"{Name}_cases.json"
        code, out, err = run_cmd([PYTHON, str(plot_mbd_script), str(json_mbd_cases)], cwd=module_dir)
        if code != 0:
            print(f"[{name}] MBD Plot generation failed:\n{err}")
            summary.append({"name": name, "step": "MBD Plot", "status": "FAIL", "msg": err})
            continue
            
        # 6. Compile LaTeX to PDF
        print(f"[{name}] Compiling LaTeX document...")
        latex_compile_script = Path(__file__).parent / "compile_latex.py"
        tex_file = module_dir / "doc" / "cpp" / f"{name}.tex"
        # Since compile_latex.py compiles tex file and automatically mapped path outputs to pdf,
        # let's run it.
        # compile_latex.py usage: python <PromptDir>/script/compile_latex.py [tex_file.tex] [output_dir]
        # output_dir defaults to same folder as tex file.
        code, out, err = run_cmd([PYTHON, str(latex_compile_script), str(tex_file)], cwd=module_dir)
        if code != 0:
            print(f"[{name}] LaTeX compile failed:\n{out}\n{err}")
            summary.append({"name": name, "step": "LaTeX", "status": "FAIL", "msg": out + err})
            continue
        else:
            print(f"[{name}] LaTeX document compiled successfully!")
            
        summary.append({"name": name, "step": "All Steps", "status": "PASS", "msg": ""})
        
    print("\n==================================================")
    print("               VERIFICATION SUMMARY               ")
    print("==================================================")
    all_passed = True
    for item in summary:
        status_symbol = "✅" if item["status"] == "PASS" else "❌"
        print(f"{status_symbol} {item['name']}: {item['step']} - {item['status']}")
        if item["status"] != "PASS":
            all_passed = False
            
    print("\nFinal Result: " + ("ALL PASSED ✅" if all_passed else "SOME FAILED ❌"))

if __name__ == '__main__':
    main()
