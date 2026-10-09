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
    ├── main.py            integrates everything in Src
    ├── Src/
    │   ├── global_parameters.py
    │   ├── <subfolder>/   e.g. Drag_Estimation, Stability
    │   │   ├── Helpers/          small reusable functions
    │   │   ├── Modules/          phases of the code (aerodynamic parameter
    │   │   │                     prediction, plotter, ...)
    │   │   └── Refactoring_Wip/  temporary: files not yet split into one
    │   │                         class/function per file
    │   └── Deprecated/    old code kept for reference only
    └── Tests/             mirrors Src, but only one level deep
        └── <subfolder>/   test files sit directly here (no Helpers/Modules)
```

- **Naming:** folder names start each word with a capital letter (`Drag_Estimation`, `Refactoring_Wip`). Code files are all lowercase (`global_parameters.py`). README files are the exception.
- **`Helpers/`** holds small functions that modules call.
- **`Modules/`** holds the phases of the code.
- **`main.py`** is the only place where modules are wired together. The final assembly of the code happens only in `main.py` of that version, never inside `Src/`.
- **One class or function per file.** Each file in `Src/` contains exactly one class or one function, and the file is named after it.
- **`Tests/`** has the same subfolders as `Src/` (for example `Tests/Drag_Estimation/`), but no `Helpers/` or `Modules/` inside them.
- **`Refactoring_Wip/`** (inside a `Src` subfolder) holds existing code that still breaks the one-class-or-function-per-file rule. It is temporary: refactor the files into `Helpers/` and `Modules/`, then delete the folder.
- **`Deprecated/`** is not maintained and has no branch of its own.

## Branch workflow

```
main  <──  dev  <──  drag_estimation branch
                <──  stability branch
                <──  (one branch per Src subfolder)
```

| Branch | Purpose | Who pushes |
|---|---|---|
| **`main`** | Most recent stable version. | Only the system owner (git admin), and only by updating it from `dev`. |
| **`dev`** | Tracks the current `Version_*`. It belongs to no one's day-to-day work: it only pulls (merges) from the subfolder branches, and is where the integrated version is assembled and checked. | Nobody commits to it directly. Merges from subfolder branches only. |
| **One branch per `Src` subfolder** (e.g. `Drag_Estimation`, `Stability`; `Deprecated` has none) | All development for that subfolder. These branches belong solely to `dev`: they are created from `dev` and only ever merge into `dev`, never into `main` or into each other. | Each contributor pushes only to their own subfolder branch. |

1. Create or update your subfolder branch from `dev`, and work there. Do not commit to `dev` or `main`.
2. Push only to your own subfolder branch.
3. When the work is approved, it is merged into `dev`. `dev` only receives merges from subfolder branches.
4. When `dev` holds a stable version, the system owner updates `main` from `dev`. `main` then always contains the latest stable version.
5. After that, the system owner clears `dev` and starts the next `Version_*` on it. The subfolder branches then sync from `dev`.

Nobody except the system owner pushes to `main`.

## Setup

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
