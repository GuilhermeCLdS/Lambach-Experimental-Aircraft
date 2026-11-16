"""
Read APC PER3 performance data + geometry file, plot the *actual* data as
3D surfaces (thrust / power / efficiency vs RPM and velocity), and verify the
choose_prop surrogate operating points against the real data.

Prop diameter, pitch and blade count come from the geometry file.

Usage:
    python prop_data_verify.py                      # looks for files in ./, ../../../data, /mnt/user-data/uploads
    python prop_data_verify.py --perf P.txt --geom G.txt
    python prop_data_verify.py --save               # write PNGs instead of showing windows
"""
import argparse
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

MPH_TO_MS = 0.44704
IN_TO_M = 0.0254

PERF_NAME = "PER3_5x37E-3_performance_file.txt"
GEOM_NAME = "5x37E-3-PERF_geometry.txt"

# --------------------------------------------------------------------------
# choose_prop surrogate operating points to verify: (label, rpm, V [m/s], T [N])
# --------------------------------------------------------------------------
SURROGATE_POINTS = [
    ("cruise",        24307, 55.6,  0.83),
    ("cruise",        27767, 55.6,  0.83),
    ("full thr @ V",  38785, 55.6,  6.22),
    ("full thr @ V",  39422, 55.6,  5.68),
    ("static",        37817,  0.0, 10.51),
    ("static",        38359,  0.0, 11.72),
]


def find_file(name, explicit=None):
    if explicit:
        return Path(explicit)
    here = Path(__file__).resolve().parent
    candidates = [Path.cwd(), here, here / "data"]
    if len(here.parents) > 2:
        candidates.append(here.parents[2] / "data")
    candidates.append(Path("/mnt/user-data/uploads"))
    for d in candidates:
        if (d / name).exists():
            return d / name
    raise FileNotFoundError(f"Could not find {name}; pass it explicitly.")


# --------------------------------------------------------------------------
# Geometry file
# --------------------------------------------------------------------------
def read_geometry(filepath):
    """Return dict with D_in, R_in, blades, P_nom_in, P_075R_in, name."""
    txt = Path(filepath).read_text()

    R_in = float(re.search(r"^\s*RADIUS:\s*([\d.]+)", txt, re.M).group(1))
    blades = int(re.search(r"^\s*BLADES:\s*(\d+)", txt, re.M).group(1))

    m = re.match(r"\s*([\d.]+)x([\d.]+)E", txt)
    P_nom = float(m.group(2)) if m else np.nan

    # station table -> quoted pitch at 0.75R as a cross-check
    rows = []
    for line in txt.splitlines():
        if re.match(r"\s*\d\.\d{4}(\s+-?\d+\.\d+){13}\s*$", line):
            rows.append([float(x) for x in line.split()])
    rows = np.array(rows)
    P_075 = float(np.interp(0.75 * R_in, rows[:, 0], rows[:, 2])) if len(rows) else np.nan

    return dict(name=txt.splitlines()[0].strip(), R_in=R_in, D_in=2 * R_in,
                blades=blades, P_nom_in=P_nom, P_075R_in=P_075)


# --------------------------------------------------------------------------
# Performance file
# --------------------------------------------------------------------------
def read_perf_file(filepath):
    """Parse PER3 file -> {rpm: dict of arrays}. Velocities converted to m/s."""
    sections, current, rows = {}, None, []

    def flush():
        if current is not None and rows:
            a = np.array(rows)
            sections[current] = dict(V_ms=a[:, 0] * MPH_TO_MS, J=a[:, 1], eta=a[:, 2],
                                     Ct=a[:, 3], Cp=a[:, 4], Power_W=a[:, 5],
                                     Torque_Nm=a[:, 6], Thrust_N=a[:, 7])

    for line in Path(filepath).read_text().splitlines():
        s = line.strip()
        if s.startswith("PROP RPM"):
            flush()
            current = int(s.split("=")[1])
            rows = []
            continue
        parts = s.split()
        if current is None or len(parts) < 14:
            continue
        try:
            v = [float(p) for p in parts[:15]]
        except ValueError:
            continue  # header lines
        # cols: V J Pe Ct Cp PWR(Hp) Torque(inlbf) Thrust(lbf) PWR(W) Torque(Nm) Thrust(N) ...
        rows.append([v[0], v[1], v[2], v[3], v[4], v[8], v[9], v[10]])
    flush()
    return sections


def infer_rho(data, D_m):
    """Back out the air density APC used from the lowest-RPM static row."""
    rpm = min(data)
    d = data[rpm]
    n = rpm / 60.0
    return d["Thrust_N"][0] / (d["Ct"][0] * n**2 * D_m**4)


# --------------------------------------------------------------------------
# Gridding (no fitted model: per-RPM np.interp onto a common V grid, NaN outside data)
# --------------------------------------------------------------------------
def grid_quantity(data, key, V_grid, positive_only=False):
    rpms = sorted(data)
    Z = np.full((len(rpms), len(V_grid)), np.nan)
    for i, rpm in enumerate(rpms):
        d = data[rpm]
        Z[i] = np.interp(V_grid, d["V_ms"], d[key], left=np.nan, right=np.nan)
    if positive_only:
        Z[Z < 0] = np.nan
    return np.array(rpms), Z


def surface_plot(data, key, zlabel, title, cmap, V_max, markers=None,
                 positive_only=False, zmin=None):
    V_grid = np.linspace(0, V_max, 120)
    rpms, Z = grid_quantity(data, key, V_grid, positive_only)
    if zmin is not None:
        Z = np.where(Z < zmin, np.nan, Z)
    RPM, V = np.meshgrid(rpms, V_grid, indexing="ij")

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(RPM, V, Z, cmap=cmap, edgecolor="none", alpha=0.85)
    ax.set_xlabel("RPM")
    ax.set_ylabel("Velocity [m/s]")
    ax.set_zlabel(zlabel)
    ax.set_title(title)
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label=zlabel)

    if markers:
        for (lab, style), pts in markers.items():
            if pts:
                p = np.array(pts)
                ax.scatter(p[:, 0], p[:, 1], p[:, 2], label=lab, depthshade=False, **style)
        ax.legend(loc="upper left")
    plt.tight_layout()
    return fig


# --------------------------------------------------------------------------
# Real-data lookup for verification: interpolate Ct(J) within each RPM,
# then linearly across RPM, then rebuild thrust. (Ct varies far more slowly
# with RPM than thrust does, so this is more accurate than interpolating T.)
# --------------------------------------------------------------------------
def real_thrust_power(data, rpm, V_ms, D_m, rho):
    n = rpm / 60.0
    J = V_ms / (n * D_m)
    rpms = np.array(sorted(data))
    ct = np.array([np.interp(J, data[r]["J"], data[r]["Ct"], left=np.nan, right=np.nan) for r in rpms])
    cp = np.array([np.interp(J, data[r]["J"], data[r]["Cp"], left=np.nan, right=np.nan) for r in rpms])
    ok = ~np.isnan(ct)
    if ok.sum() < 2 or not (rpms[ok].min() <= rpm <= rpms[ok].max()):
        return J, np.nan, np.nan, np.nan, np.nan
    ct_i = np.interp(rpm, rpms[ok], ct[ok])
    cp_i = np.interp(rpm, rpms[ok], cp[ok])
    T = ct_i * rho * n**2 * D_m**4
    P = cp_i * rho * n**3 * D_m**5
    return J, ct_i, cp_i, T, P


def verify(data, D_m, rho):
    print("\n=== VERIFICATION: choose_prop surrogate vs real APC data ===")
    print(f"{'case':<14}{'RPM':>7}{'V[m/s]':>8}{'J':>7}{'Ct':>8}{'Cp':>8}"
          f"{'T_real[N]':>11}{'T_surr[N]':>11}{'err[%]':>9}{'P_real[W]':>11}")
    real_pts = []
    for lab, rpm, V, T_s in SURROGATE_POINTS:
        J, ct, cp, T, P = real_thrust_power(data, rpm, V, D_m, rho)
        if np.isnan(T):
            r_near = min(data, key=lambda r: abs(r - rpm))
            print(f"{lab:<14}{rpm:7d}{V:8.1f}{J:7.3f}   -- J beyond end of table (J_max={data[r_near]['J'].max():.3f} at {r_near} RPM): "
                  f"prop is at/below zero thrust, surrogate T={T_s:.2f} N is not achievable --")
            continue
        err = (T_s - T) / T * 100 if abs(T) > 1e-9 else np.nan
        print(f"{lab:<14}{rpm:7d}{V:8.1f}{J:7.3f}{ct:8.4f}{cp:8.4f}{T:11.3f}{T_s:11.3f}{err:9.1f}{P:11.1f}")
        real_pts.append((rpm, V, T))
    return real_pts


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--perf")
    ap.add_argument("--geom")
    ap.add_argument("--vmax", type=float, default=None, help="max velocity [m/s] to plot (default: from data)")
    ap.add_argument("--save", action="store_true", help="save PNGs instead of showing")
    args = ap.parse_args()

    if args.save:
        plt.switch_backend("Agg")

    geom = read_geometry(find_file(GEOM_NAME, args.geom))
    D_m = geom["D_in"] * IN_TO_M
    print(f"Prop {geom['name']}: D={geom['D_in']:.2f} in ({D_m:.4f} m), blades={geom['blades']}, "
          f"nominal pitch={geom['P_nom_in']} in, pitch@0.75R={geom['P_075R_in']:.3f} in")

    data = read_perf_file(find_file(PERF_NAME, args.perf))
    rpms = sorted(data)
    print(f"Found {len(rpms)} RPM sections: {rpms[0]}..{rpms[-1]} RPM")

    rho = infer_rho(data, D_m)
    print(f"Air density implied by file: {rho:.4f} kg/m^3")

    V_max = args.vmax or max(d["V_ms"].max() for d in data.values())
    title_base = f"APC {geom['D_in']:.0f}x{geom['P_nom_in']} (3-blade, PER3 data)"

    real_pts = verify(data, D_m, rho)

    # markers
    kinds = {
        "cruise":       ("choose_prop cruise",        dict(color="red", s=70)),
        "full thr @ V": ("choose_prop full throttle", dict(color="orange", s=70)),
        "static":       ("choose_prop static",        dict(color="blue", s=70)),
    }
    markers = {}
    for lab, rpm, V, T in SURROGATE_POINTS:
        name, style = kinds[lab]
        markers.setdefault(name, (style, []))[1].append((rpm, V, T))

    figs = {}
    V_grid_label = "Velocity [m/s]"

    def thrust_fig():
        V_grid = np.linspace(0, V_max, 120)
        rp, Z = grid_quantity(data, "Thrust_N", V_grid, positive_only=True)
        RPM, V = np.meshgrid(rp, V_grid, indexing="ij")
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection="3d")
        surf = ax.plot_surface(RPM, V, Z, cmap="viridis", edgecolor="none", alpha=0.85)
        ax.set_xlabel("RPM"); ax.set_ylabel(V_grid_label); ax.set_zlabel("Thrust [N]")
        ax.set_title(f"{title_base}\nThrust from performance file")
        fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label="Thrust [N]")
        for n, (s, pts) in markers.items():
            p = np.array(pts)
            ax.scatter(p[:, 0], p[:, 1], p[:, 2], label=n, depthshade=False, **s)
        if real_pts:
            p = np.array(real_pts)
            ax.scatter(p[:, 0], p[:, 1], p[:, 2], color="green", marker="^", s=90,
                       label="real data (same RPM, V)", depthshade=False)
        ax.legend(loc="upper left")
        plt.tight_layout()
        return fig

    figs["thrust"] = thrust_fig()
    figs["power"] = surface_plot(data, "Power_W", "Shaft power [W]",
                                 f"{title_base}\nPower from performance file", "plasma", V_max)
    figs["efficiency"] = surface_plot(data, "eta", "Propulsive efficiency",
                                      f"{title_base}\nEfficiency from performance file", "coolwarm",
                                      V_max, positive_only=True)

    if args.save:
        for k, f in figs.items():
            out = f"prop_{k}_surface.png"
            f.savefig(out, dpi=130)
            print("saved", out)
    else:
        plt.show()


if __name__ == "__main__":
    main()