"""
check_env.py — Is this machine ready for the robotics labs?

Run it at the start of every lab session (it takes a few seconds):

    conda activate zuj-robotics
    python check_env.py          # quick check, no window
    python check_env.py --gui    # also opens the PyBullet 3D window for 3 s

Every line should say [OK]. If one says [FAIL], read the hint under it,
or see the Troubleshooting section of SETUP.md.
"""

import importlib
import importlib.metadata
import os
import sys
import time
from contextlib import contextmanager

# The versions the course is tested with (see environment.yml).
REQUIRED_PYTHON = (3, 11)
PACKAGES = [
    # (import name, name to show, why the course needs it)
    ("numpy", "NumPy", "matrices for the Kalman filter and the maps"),
    ("matplotlib", "Matplotlib", "trajectory and map plots"),
    ("pybullet", "PyBullet", "the physics simulator"),
    ("pybullet_data", "pybullet_data", "robot and floor models"),
    ("cv2", "OpenCV", "camera images"),
]

failures = 0


def ok(message):
    print(f"  [OK]   {message}")


def fail(message, hint):
    global failures
    failures += 1
    print(f"  [FAIL] {message}")
    print(f"         -> {hint}")


@contextmanager
def quiet_c_output():
    """PyBullet's C++ core prints dozens of URDF warnings straight to the
    console, bypassing Python. Point the console (file descriptor 1) at the
    null device for a moment so our [OK] / [FAIL] lines stay readable.
    (One "pybullet build time" line may still appear on Windows; it's harmless.)"""
    sys.stdout.flush()
    saved = os.dup(1)
    null = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 1)
    try:
        yield
    finally:
        sys.stdout.flush()
        os.dup2(saved, 1)
        os.close(null)
        os.close(saved)


def package_version(module, module_name):
    # PyBullet has no __version__, so fall back to the installed package info.
    try:
        return getattr(module, "__version__", "") or importlib.metadata.version(module_name)
    except importlib.metadata.PackageNotFoundError:
        return ""


print("\nChecking your robotics environment...\n")

# ---------------------------------------------------------------------------
# 1. Are we inside the course environment?
#    The most common mistake is forgetting "conda activate zuj-robotics",
#    which runs the labs with a different Python that has none of the packages.
# ---------------------------------------------------------------------------
env_name = os.path.basename(os.path.normpath(sys.prefix))
if env_name == "zuj-robotics":
    ok(f"Environment: zuj-robotics ({sys.prefix})")
else:
    fail(f"Environment: this Python belongs to '{env_name}', not zuj-robotics",
         "Run: conda activate zuj-robotics  (on lab PCs: use the Robotics Lab shortcut)")

# ---------------------------------------------------------------------------
# 2. Python version
# ---------------------------------------------------------------------------
version = sys.version_info
if (version.major, version.minor) == REQUIRED_PYTHON:
    ok(f"Python {version.major}.{version.minor}.{version.micro}")
else:
    fail(f"Python {version.major}.{version.minor} (course uses 3.11)",
         "Recreate the environment: conda env create -f environment.yml")

# ---------------------------------------------------------------------------
# 3. Packages: can each one be imported, and from where?
# ---------------------------------------------------------------------------
for module_name, label, purpose in PACKAGES:
    try:
        with quiet_c_output():
            module = importlib.import_module(module_name)
        found_version = package_version(module, module_name)
        ok(f"{label} {found_version}".rstrip() + f"  ({purpose})")
    except Exception as error:  # ImportError, or a broken DLL on Windows
        fail(f"{label} cannot be imported: {error}",
             "Run: conda env update -f environment.yml --prune")

# ---------------------------------------------------------------------------
# 4. Packages from "pip install --user" can hide the course versions.
# ---------------------------------------------------------------------------
if sys.flags.no_user_site or os.environ.get("PYTHONNOUSERSITE"):
    ok("User site-packages ignored (no leftovers from other installs)")
else:
    import site
    user_site = site.getusersitepackages()
    if os.path.isdir(user_site) and os.listdir(user_site):
        fail(f"Packages found in {user_site}",
             "They may override the course versions. Re-activate zuj-robotics, "
             "or remove that folder if you don't need it.")
    else:
        ok("No user site-packages")

# ---------------------------------------------------------------------------
# 5. Can PyBullet actually simulate? Spawn the floor and the Husky robot
#    used in the labs, then step the physics for one simulated second.
# ---------------------------------------------------------------------------
use_gui = "--gui" in sys.argv
try:
    import pybullet as p
    import pybullet_data

    with quiet_c_output():
        client = p.connect(p.GUI if use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        p.loadURDF("husky/husky.urdf", [0, 0, 0.1])
        for _ in range(240):  # 240 steps x 1/240 s = 1 simulated second
            p.stepSimulation()
            if use_gui:
                time.sleep(3 / 240)  # slow it down so you can see the robot
        p.disconnect(client)
    mode = "3D window" if use_gui else "headless"
    ok(f"PyBullet spawned the Husky robot and simulated 1 s ({mode})")
except Exception as error:
    hint = ("On a remote or headless machine run without --gui."
            if use_gui else "Run: conda env update -f environment.yml --prune")
    fail(f"PyBullet simulation failed: {error}", hint)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print()
if failures == 0:
    print("All checks passed. You are ready for the lab!\n")
else:
    print(f"{failures} check(s) failed. Fix them before starting the lab.\n")
sys.exit(1 if failures else 0)
