# Lambach-Experimental-Aircraft
A collection of the code developed by the Lambach Experimental Aircraft team.

## Repository structure

```
Lambach-Experimental-Aircraft/
├── README.md
├── requirements.txt
├── docs/                  plots, write-ups, figures
├── data/                  csvs and other datasets
└── version_*/             one folder per version (version_1, version_2, ...)
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

- **Naming:** everything is lowercase, with words separated by underscores: folders (`drag_estimation`, `refactoring_wip`) and code files (`global_parameters.py`). README files are the exception.
- **`helpers/`** holds small functions that modules call.
- **`modules/`** holds the phases of the code.
- **`main.py`** is the only place where modules are wired together. The final assembly of the code happens only in `main.py` of that version, never inside `src/`.
- **One class or function per file.** Each file in `src/` contains exactly one class or one function, and the file is named after it.
- **`tests/`** has the same subfolders as `src/` (for example `tests/drag_estimation/`), but no `helpers/` or `modules/` inside them.
- **`refactoring_wip/`** (inside a `src` subfolder) holds existing code that still breaks the one-class-or-function-per-file rule. It is temporary: refactor the files into `helpers/` and `modules/`, then delete the folder.
- **`deprecated/`** is not maintained and has no branch of its own.

## Branch workflow

```
main  <──  dev  <──  drag_estimation branch
                <──  stability branch
                <──  (one branch per src subfolder)
```

| Branch | Purpose | Who pushes |
|---|---|---|
| **`main`** | Most recent stable version. | Only the system owner (git admin), and only by updating it from `dev`. |
| **`dev`** | Tracks the current `version_*`. It belongs to no one's day-to-day work: it only pulls (merges) from the subfolder branches, and is where the integrated version is assembled and checked. | Nobody commits to it directly. Merges from subfolder branches only. |
| **One branch per `src` subfolder** (e.g. `drag_estimation`, `stability`; `deprecated` has none) | All development for that subfolder. These branches belong solely to `dev`: they are created from `dev` and only ever merge into `dev`, never into `main` or into each other. | Each contributor pushes only to their own subfolder branch. |

1. Create or update your subfolder branch from `dev`, and work there. Do not commit to `dev` or `main`.
2. Push only to your own subfolder branch.
3. When the work is approved, it is merged into `dev`. `dev` only receives merges from subfolder branches.
4. When `dev` holds a stable version, the system owner updates `main` from `dev`. `main` then always contains the latest stable version.
5. After that, the system owner clears `dev` and starts the next `version_*` on it. The subfolder branches then sync from `dev`.

Nobody except the system owner pushes to `main`.

## Setup

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
