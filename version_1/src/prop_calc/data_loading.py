"""Load APC PER3 propeller performance files from disk.

PER3 files contain tabulated performance data (V, J, Pe, Ct, Cp, ...) at
multiple RPM sections. This module parses those files into Prop objects.
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np

from prop import Prop

# File patterns for APC PER3 performance data (exclude geometry files).
PERF_GLOBS = ("PER3_*.txt", "PER3_*.dat")

# Header pattern: e.g. "5x3.7E-3" or "5x4E" or "5x4E-4" (diameter x pitch, optional type, optional blades).
_HEADER_RE = re.compile(r"\s*(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)([A-Za-z]*)(?:-(\d+))?")


def _perf_files(data_dir: str | Path) -> list[Path]:
    """Return sorted list of PER3 performance files in data_dir (recursive)."""
    files: list[Path] = []
    for pattern in PERF_GLOBS:
        for path in sorted(Path(data_dir).rglob(pattern)):
            if "geometry" not in path.name.lower():
                files.append(path)
    return files


def read_perf_file(filepath: str | Path, rpm_table_tol: float = 1.01) -> Prop:
    """Parse one PER3 performance file into a Prop.

    Args:
        filepath: Path to a PER3 *.txt/*.dat file.
        rpm_table_tol: Tolerance factor for maximum RPM (passed to Prop).

    Returns:
        Prop instance with parsed diameter, pitch, blade count, and Ct/Cp tables.

    Raises:
        ValueError: If the header cannot be parsed or fewer than 2 RPM sections exist.
        StopIteration: If the file has no non-empty header line.
    """
    lines = Path(filepath).read_text().splitlines()

    # First non-empty line is the header (e.g. "5x3.7E-3  ...").
    header = next(line for line in lines if line.strip())
    match = _HEADER_RE.match(header)
    if not match:
        raise ValueError(f"cannot read prop size from header: {header!r}")

    d_in = float(match.group(1))
    p_in = float(match.group(2))
    blades = int(match.group(4)) if match.group(4) else 2
    label = header.split()[0]

    sections: dict[int, dict[str, np.ndarray]] = {}
    current_rpm: int | None = None
    rows: list[list[float]] = []

    def flush_section() -> None:
        """Store the accumulated rows for the current RPM section."""
        nonlocal rows
        if current_rpm is not None and len(rows) > 3:
            arr = np.array(rows)
            sections[current_rpm] = {"J": arr[:, 0], "Ct": arr[:, 1], "Cp": arr[:, 2]}
        rows = []

    for line in lines:
        stripped = line.strip()

        # Start of a new RPM section.
        if stripped.startswith("PROP RPM"):
            flush_section()
            current_rpm = int(stripped.split("=")[1])
            continue

        # Data rows begin only after we've seen an RPM header.
        if current_rpm is None:
            continue

        parts = stripped.split()
        if len(parts) < 14:
            continue

        # Columns: V(mph), J, Pe, Ct, Cp, ...
        try:
            values = [float(x) for x in parts[:5]]
        except ValueError:
            continue

        rows.append([values[1], values[3], values[4]])  # J, Ct, Cp

    flush_section()

    if len(sections) < 2:
        raise ValueError("fewer than 2 RPM sections parsed")

    return Prop(label, d_in, p_in, blades, sections, rpm_table_tol)


def load_props(data_dir: str | Path, rpm_table_tol: float = 1.01) -> list[Prop]:
    """Load all unique PER3 propellers from a directory.

    Skips files that cannot be parsed and duplicate prop labels (with warnings).
    """
    props: list[Prop] = []
    seen_labels: set[str] = set()

    for filepath in _perf_files(data_dir):
        try:
            prop = read_perf_file(filepath, rpm_table_tol)
        except (ValueError, StopIteration) as exc:
            print(f"  skipped {filepath.name}: {exc}")
            continue

        if prop.label in seen_labels:
            print(f"  skipped {filepath.name}: duplicate of {prop.label}")
            continue

        seen_labels.add(prop.label)
        props.append(prop)

    return props
