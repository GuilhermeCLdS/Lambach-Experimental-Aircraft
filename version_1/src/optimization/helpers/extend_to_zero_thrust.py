import numpy as onp

J_COLUMN, CT_COLUMN, CP_COLUMN = 1, 3, 4


def extend_to_zero_thrust(block, n_fit=3):
    """Return (J, Ct, Cp) for one APC RPM block, extended linearly to Ct = 0.

    APC stops each block close to, but not always at, zero thrust. A straight
    line through the last ``n_fit`` points gives the zero-thrust advance ratio
    J0; if J0 lies beyond the last data point, (J0, 0, Cp(J0)) is appended.
    Blocks that already reach zero thrust are returned unchanged.
    """
    j, ct, cp = block[:, J_COLUMN], block[:, CT_COLUMN], block[:, CP_COLUMN]
    ct_slope, ct_icpt = onp.polyfit(j[-n_fit:], ct[-n_fit:], 1)
    if ct[-1] <= 0 or ct_slope >= 0:
        return j, ct, cp

    j_zero = -ct_icpt / ct_slope
    if j_zero <= j[-1]:
        return j, ct, cp

    cp_slope, cp_icpt = onp.polyfit(j[-n_fit:], cp[-n_fit:], 1)
    return (
        onp.append(j, j_zero),
        onp.append(ct, 0.0),
        onp.append(cp, cp_slope * j_zero + cp_icpt),
    )
