# Setting Up Your Robotics Environment

[النسخة العربية](SETUP.ar.md)

Every lab and the final project use **one environment called `zuj-robotics`**.
It has the same name and the same package versions on your laptop, your home PC
and the university lab PCs, so code that runs in one place runs everywhere.

You can work in three ways. Pick whichever suits you; you can switch at any time.

| Where | Setup needed | 3D simulator window |
|-------|--------------|:-------------------:|
| **Google Colab** | None. Click the Colab link of any lab | No (plots only) |
| **University lab PC** | None. It is already installed | Yes |
| **Your own laptop or PC** | About 15 minutes, once (this guide) | Yes |

> **Do not create your own environment on the lab PCs** (for example
> `robotics_YourName`). The shared `zuj-robotics` environment is already there
> on every lab PC. See [On the university lab PCs](#on-the-university-lab-pcs).

---

## On your own laptop or PC

Works on Windows 10/11, macOS (Intel and Apple Silicon) and Linux.
You need about 3 GB of free disk space and an internet connection for the first setup.

### Step 1: Install Miniforge (once)

Miniforge is a small, free installer for `conda`, the tool that builds the
environment. **Already have Anaconda or Miniconda?** Skip to Step 2 and use your
Anaconda Prompt. The course file only uses the free `conda-forge` channel.

**Windows**

1. Download `Miniforge3-Windows-x86_64.exe` from
   [github.com/conda-forge/miniforge](https://github.com/conda-forge/miniforge#download).
2. Run it and keep the defaults ("Just Me", default folder).
3. Open **Miniforge Prompt** from the Start menu. Use this window for every command below.

**macOS and Linux**: open a Terminal and run:

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

Answer `yes` to the licence and to "initialize conda", then **close and reopen the Terminal**.

### Step 2: Get the course code

With git (recommended, because `git pull` then fetches each new lab):

```bash
git clone https://github.com/akramfatayer/zuj-robotics.git
cd zuj-robotics
```

Without git: on the [repository page](https://github.com/akramfatayer/zuj-robotics)
click **Code → Download ZIP**, unzip it, then `cd` into the unzipped folder.

### Step 3: Create the environment (once)

From inside the `zuj-robotics` folder:

```bash
conda env create -f environment.yml
```

This downloads Python 3.11, PyBullet, NumPy, Matplotlib and OpenCV. It takes
5–10 minutes the first time.

### Step 4: Check it works

```bash
conda activate zuj-robotics
python check_env.py --gui
```

A PyBullet window opens for a few seconds with a Husky robot, and every line
says `[OK]`. You're ready.

### Every time you work on a lab

```bash
conda activate zuj-robotics
cd zuj-robotics
git pull                              # get the latest lab files
cd labs/lab-01-hello-robot            # the lab you are working on
python spawn_robot.py
```

### Using VS Code (optional)

Open the `zuj-robotics` folder, press `Ctrl+Shift+P` (`Cmd+Shift+P` on Mac),
choose **Python: Select Interpreter**, and pick **zuj-robotics**.

### When the instructor announces an update

```bash
conda activate zuj-robotics
conda env update -f environment.yml --prune
```

---

## On the university lab PCs

The environment is already installed. You don't need to install anything.

1. Double-click **Robotics Lab** on the desktop. A terminal opens with
   `zuj-robotics` already active and runs `check_env.py` for you.
2. Get your work (see below), then run the lab as usual.

**Your work does not stay on the lab PC.** You may sit at a different PC next
time, and the lab PCs are shared with the other section. Before you leave, save
your files in one of these:

- **Your own GitHub repository** (best): `git push` at the end, `git pull` at the start.
- **OneDrive or Google Drive**: upload your lab folder at the end of the session.
- A USB stick, as a backup only.

**Please don't run `pip install` or `conda install` on the lab PCs.** The shared
environment is read-only so that it works the same for everyone. If something
seems to be missing, tell the instructor.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| A line `pybullet build time: ...` appears | Normal. PyBullet prints it every time it starts. |
| `conda` is not recognized | Windows: use **Miniforge Prompt** (or Anaconda Prompt), not the normal Command Prompt. macOS/Linux: close and reopen the Terminal. |
| `ModuleNotFoundError: No module named 'pybullet'` | The environment isn't active. Run `conda activate zuj-robotics`. |
| `check_env.py` says the Python belongs to `base` or another name | Same fix: `conda activate zuj-robotics`. |
| `EnvironmentNameNotFound` when activating | Step 3 didn't finish. Run `conda env create -f environment.yml` again from the `zuj-robotics` folder. |
| `pip install pybullet` fails with "Microsoft Visual C++ 14.0 is required" | Don't use pip for PyBullet. The environment file installs a ready-made build from conda-forge. |
| The PyBullet window doesn't open | Run `python check_env.py` (no `--gui`). If that passes, your graphics driver blocks the window. Update it, or use the lab PCs or Colab for the 3D view. |
| Something is broken and you want to start fresh | `conda env remove -n zuj-robotics`, then repeat Step 3. |
| You still have last semester's `robotics_YourName` environment | You can delete it: `conda env remove -n robotics_YourName`. |

Still stuck? Post the **full output of `python check_env.py`** in the
*Ask the Robot Lab* forum on Moodle.
