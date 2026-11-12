import re

import numpy as onp

RPM_HEADER = re.compile(r"PROP RPM =\s*([\d.]+)")
N_COLUMNS = 15


def read_apc_performance_blocks(path):
    """Parse an APC PER3 file into {rpm: rows} with one numeric array per RPM.

    Each row has the 15 PER3 columns: V, J, Pe, Ct, Cp, PWR, Torque, Thrust,
    PWR(W), Torque(Nm), Thrust(N), THR/PWR, Mach, Reyn, FOM.
    """
    blocks = {}
    current_rpm = None

    with open(path, "r", errors="ignore") as file:
        for line in file:
            match = RPM_HEADER.search(line)
            if match:
                current_rpm = float(match.group(1))
                blocks[current_rpm] = []
                continue

            parts = line.split()
            if current_rpm is not None and len(parts) == N_COLUMNS:
                try:
                    blocks[current_rpm].append([float(value) for value in parts])
                except ValueError:
                    continue

    return {rpm: onp.asarray(rows, dtype=float) for rpm, rows in blocks.items()}
