# COURSE_CONTEXT — Robotics

> Living summary of this course. Keep it short and current — it exists so any AI (or person) can re-orient in under a minute. Update it as a proposed change whenever something meaningful shifts.

## Snapshot

- **Course:** Intelligent Mobile Robots / Introduction to Robotics for AI and Data Science (0135343)
- **Version:** v2 (semester 20252)
- **Status:** stable (materials complete, migrating to hub)
- **Last meaningful update:** 2026-06-17

## Purpose & audience

For AI and Data Science undergraduates with Python and ML experience but no robotics background. The course uses a data-first, project-based approach: students progressively build a fully autonomous warehouse robot by integrating perception, localization, mapping, planning, and control.

## Structure

The course follows a progressive "Sense-Plan-Act" architecture across four project blocks:

| Block | Focus | Lectures | Labs |
|-------|-------|----------|------|
| 1 — Perception | Intro, sensors | Lecture 1 | Lab 1 |
| 2 — Localization & Mapping | Kalman filter, LiDAR, grids | Lectures 2–3 | Labs 3–4 |
| 3 — Planning | A*, RRT | Lecture 4 | Lab 5 |
| 4 — Control | PID, Pure Pursuit | Lecture 5 | Lab 6 |
| Final | Full integration | — | Final Project |

Each lab builds one layer of the autonomous pipeline. The final project wires all layers together.

## Current focus

Migrating existing Moodle materials to the `zuj-robotics` GitHub repository and MkDocs site structure. All lectures, labs, exam materials, and the final project are complete and have been delivered in semester 20252.

## Open questions

- Should PPTX lectures be converted to Markdown for the site, or served as downloadable files?
- Should the video (Robotics-01.mp4, 17 MB) be hosted on the site or linked from an external source?
- What fills the slot freed by the computer vision lecture (2026/27)?

## Decisions

- (2026-06-17) — Course materials migrated from Moodle export to organized directory structure.
- (2026-06-17) — Repository structure: `lectures/`, `labs/lab-NN-topic/`, `assessments/`, `project/`, `resources/`.
- (2026-06-17) — Lab READMEs serve as the primary lab handouts (already in Markdown).
- (2026-06-17) — Assessment model: 60% PBL (labs + final project), 40% final exam.
- (2026-10-26) — Computer vision lecture dropped: students take it in Computer Vision (0135326), and no lab, project stage or exam question used it. Localization becomes Lecture 2; lectures renumber 1–5. The CV deck stays in `lectures/` but is no longer listed. Lab numbers are unchanged (there is still no Lab 2).
- (2026-10-26) — Lecture 2 (localization) rebuilt around one storyline: uncertainty → belief → motion alone (odometry drives the predict step as the control input) → sensing alone (GPS, the update step) → a first idea that fails (the gated reset) → weighing by uncertainty (the Kalman gain) → the Kalman filter → 2D with matrices. 1D and 2D worked examples match interactive module 2. Lab 3 Activity 2 still treats odometry as a velocity measurement; the slides note both conventions.
- (2026-10-26) — Planned: a new Lecture 3, Motion Models (velocity and odometry models, the banana-shaped uncertainty, the EKF idea), with its own lab; mapping, planning and control move to Lectures 4–6. Not built yet. Lab numbers stay as they are until then.

## Conventions

- **Branch flow:** `main` → `develop` → `feature/[task]`
- **File naming:** Labs as `lab-NN-topic/`, lectures by descriptive name
- **Tone:** Academic but accessible; explain the "why" behind the "what"
- **Code:** Extensive comments explaining concepts; educational clarity over brevity
- **Student-facing materials:** Avoid mentioning specific week numbers; use "Block N" or "Lab N"
- **Learning outcomes:** Every lecture/lab maps to program learning outcomes (MK1, MK2, MS1, MS2, MC1, MC2, MT1)
- **Tools:** Python 3.11, Anaconda, PyBullet, NumPy, Matplotlib, OpenCV
- **Simulation:** PyBullet with Husky and R2D2 robot models in warehouse environment
