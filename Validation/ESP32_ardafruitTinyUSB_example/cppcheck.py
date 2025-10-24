Import("env")
import subprocess

# Run Cppcheck on 'src' and 'include' folders
def run_cppcheck(source, target, env):
    print("Running Cppcheck...")
    result = subprocess.run([
        "cppcheck",
        "--enable=all",          # enable all checks
        "--inconclusive",        # include inconclusive checks
        "--quiet",               # suppress normal output
        "--std=c++17",           # or c++11/c++20 depending on your project
        "--language=c++",
        "src", "include"
    ])
    if result.returncode != 0:
        print("Cppcheck found issues!")

# Hook Cppcheck before compiling
env.AddPreAction("buildprog", run_cppcheck)
