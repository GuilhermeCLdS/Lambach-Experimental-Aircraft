# Lambach-Experimental-Aircraft
A collection of the code developed by the Lambach Experimental Aircraft team.

## Repository structure

```
Lambach-Experimental-Aircraft/
├── README.md
├── requirements.txt
├── Docs/                  plots, write-ups, figures
├── Data/                  csvs and other datasets
└── Version_*/             one folder per version (Version_1, Version_2, ...)
    ├── main.py            integrates everything in src
    ├── src/
    │   ├── global_parameters.py
    │   ├── <subfolder>/   e.g. drag_estimation, stability
    │   │   ├── helpers/          small reusable functions
    │   │   ├── modules/          phases of the code (aerodynamic parameter
    │   │   │                     prediction, plotter, ...)
    │   │   └── refactoring_wip/  temporary: files not yet split into one
    │   │                         class/function per file
    │   └── deprecated/    old code kept for reference only
    └── tests/             mirrors src, but only one level deep
        └── <subfolder>/   test files sit directly here (no helpers/modules)
```

- **`helpers/`** holds small functions that modules call.
- **`modules/`** holds the phases of the code.
- **`main.py`** is the only place where modules are wired together. The final assembly of the code happens only in `main.py` of that version, never inside `src/`.
- **One class or function per file.** Each file in `src/` contains exactly one class or one function, and the file is named after it.
- **`tests/`** has the same subfolders as `src/` (for example `tests/drag_estimation/`), but no `helpers/` or `modules/` inside them.
- **`refactoring_wip/`** (inside a `src` subfolder) holds existing code that still breaks the one-class-or-function-per-file rule. It is temporary: refactor the files into `helpers/` and `modules/`, then delete the folder.
- **`deprecated/`** is not maintained and has no branch of its own.

## Branch workflow

| Branch | Purpose |
|---|---|
| **One branch per `src` subfolder** (e.g. one for `drag_estimation`, one for `stability`) | All development for that subfolder happens here. `deprecated` is the only subfolder without a branch. |
| **`dev`** | Tracks the current `Version_*`. It pulls (merges) the work from the subfolder branches, so it is where the integrated version is assembled and checked. |
| **`main`** | Most recent stable version. Only the git admin touches it. |

1. Work on your own subfolder branch. Do not commit aerodynamics, stability or other work straight to `main` or `dev`.
2. When a piece of work is approved, it is merged into `dev`.
3. When `dev` holds a stable version, the git admin pushes `dev` into `main`. `main` then always contains the latest stable version.
4. After that, the admin clears `dev` and starts the next `Version_*` on it. The subfolder branches pull from `main` to stay up to date.

Nobody except the git admin pushes to `main`.

## Setup

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
