# Lab PC Setup (instructor and IT)

**Goal:** every lab PC has the *same* `zuj-robotics` environment, installed once
before the semester, shared by both sections and every student account.
Students never create or name an environment; they double-click **Robotics Lab**
on the desktop.

Why: with one environment per student (`robotics_StudentName`), the setup
was repeated on every PC a student sat at, for every section. A single
read-only environment per PC removes that work, and nobody can break it for others.

The plan: build the environment **once** on one PC, pack it into a single file
with [conda-pack](https://conda.github.io/conda-pack/), and copy it to every PC.
No internet is needed on the lab PCs, and no Anaconda install either.

---

## 1. Build the package (once per semester, on any Windows PC with conda)

In an Anaconda or Miniforge Prompt, from the root of this repository:

```bash
conda env create -f environment.yml
conda activate zuj-robotics
python check_env.py --gui            # all [OK], robot window appears

conda install -n base -c conda-forge conda-pack -y
conda pack -n zuj-robotics -o lab-pc\zuj-robotics.tar.gz
copy check_env.py lab-pc\
```

Optionally run one lab script of each block before packing, as a smoke test.

For reference (October 2026 build): the environment is about 1.4 GB, the
packed file about 480 MB, and extraction takes under a minute on an SSD.

`lab-pc\` now holds everything a lab PC needs, and goes to the lab supervisor as it is:

```
lab-pc\
  START HERE.txt           one-page steps for the supervisor (update size and SHA256 after a rebuild)
  Lab-PC-Setup-Guide.pdf   the full supervisor guide, exported from the course docs
  zuj-robotics.tar.gz      the packed environment (do not commit to git)
  check_env.py             environment checker
  deploy_lab_pc.bat        installer, run on each PC
  verify_lab_pc.bat        student-account check, copied to C:\envs by the installer
  Robotics-Lab.bat         desktop shortcut, installed by deploy_lab_pc.bat
```

This README is for the instructor: leave it out of the supervisor's copy.

## 2. Install on each lab PC (about 2 minutes per PC)

Copy the `lab-pc\` folder to a USB stick or a network share. On each PC:
right-click `deploy_lab_pc.bat` → **Run as administrator**.

The script:

1. extracts the environment to `C:\envs\zuj-robotics`
2. runs `conda-unpack` to fix the paths for that PC
3. puts **Robotics Lab** on the Public Desktop (visible to every account)
4. makes `C:\envs` read-only for the `Users` group
5. runs `check_env.py` and reports `This PC is ready.`

Then log in as a student and double-click `C:\envs\verify_lab_pc.bat`: it checks the
shortcut, that the account is a standard one, that `C:\envs` is read-only, every package
(with the 3D window) and a plot window, and ends with `This PC passed.`

If IT uses disk images, deploy on the master PC once and capture the image instead.

## 3. Each lab session

Students double-click **Robotics Lab**. The terminal opens with the
environment active, runs the checker and reminds them to save their work off the PC.
A PC that fails the check is fixed by running `deploy_lab_pc.bat` again.

## Updating mid-semester

Change `environment.yml`, rebuild the package (step 1), re-run
`deploy_lab_pc.bat` on each PC, and announce
`conda env update -f environment.yml --prune` to students with their own laptops.

## Notes

- The package is built for Windows x64 and works only on Windows x64 PCs.
- `PYTHONNOUSERSITE=1` (set by the shortcut and in `environment.yml`) stops
  `pip install --user` leftovers in student profiles from overriding course packages.
- Students who prefer their own laptops follow [SETUP.md](../SETUP.md); they get the
  same environment name and package versions from `environment.yml`.
