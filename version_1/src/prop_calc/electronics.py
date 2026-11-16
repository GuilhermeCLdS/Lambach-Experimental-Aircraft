"""Electrical model for battery, ESC, and brushless motor (quad setup).

Simple DC motor model:
    Vm = I * Rm + omega / Kv_rad_s      (motor terminal voltage)
    I  = Q * Kv_rad_s + I0              (motor current from torque)
    Vbat = I * R_pack + Vm              (at full throttle)

Where:
    Kv_rad_s = KV_rpm_per_V * 2*pi / 60
    omega = 2*pi*n (rad/s), n in rev/s
    Q = propeller torque [Nm]

Quad assumptions (default num_motors=4):
    - ESC current limit applies per motor.
    - Battery C-rate limit is shared across all motors.
    - R_PACK is per-motor equivalent resistance (battery IR/4 + wiring).
"""

from __future__ import annotations

import math

from scipy.optimize import brentq

from physics import thrust as prop_thrust, torque as prop_torque


class ElectricalSystem:
    """Container for electrical system parameters and solvers."""

    def __init__(
        self,
        cells: int = 6,
        cap_ah: float = 1.6,
        c_rate: float = 160,
        v_cell: float = 3.7,
        r_pack: float = 0.025,
        esc_a: float = 60.0,
        esc_burst_a: float = 65.0,
        esc_burst_s: float = 5.0,
        esc_eff: float = 0.95,
        num_motors: int = 4,
        sag_factor: float = 0.75,
        kv_load_factor: float = 0.8,
    ):
        self.cells = cells
        self.cap_ah = cap_ah
        self.c_rate = c_rate
        self.v_cell = v_cell
        self.r_pack = r_pack
        self.esc_a = esc_a
        self.esc_burst_a = esc_burst_a
        self.esc_burst_s = esc_burst_s
        self.esc_eff = esc_eff
        self.num_motors = num_motors
        self.sag_factor = sag_factor
        self.kv_load_factor = kv_load_factor

        # Derived
        self.vbat = cells * v_cell
        self.vbat_sag = cells * v_cell * sag_factor
        self.i_lim_batt = (c_rate * cap_ah) / num_motors
        self.i_lim_cont = min(esc_a, self.i_lim_batt)
        self.i_lim_burst = min(esc_burst_a, (c_rate * cap_ah * 2) / num_motors)

    @property
    def i_lim_per_motor(self) -> float:
        """Continuous current limit per motor."""
        return self.i_lim_cont

    def motor_current(self, prop, kv_rpm_per_v: float, i0_a: float, n_rps: float, v_mps: float) -> float:
        """Motor current [A] at operating point (n_rps, v_mps)."""
        q_nm = prop_torque(prop, n_rps, v_mps)
        kvr = kv_rpm_per_v * 2.0 * math.pi / 60.0 * self.kv_load_factor
        return q_nm * kvr + i0_a

    def full_throttle_solve(
        self, prop, kv_rpm_per_v: float, rm_ohm: float, i0_a: float, v_mps: float, vbat: float = None
    ) -> tuple[float, float, float]:
        """Solve full-throttle operating point at airspeed v_mps.

        Solves I * (Rm + R_pack) + omega / Kv_rad = Vbat for RPM.
        RPM is clamped to the prop's data-table max to avoid extrapolation.

        Returns: (n_rps, thrust_n, current_a)
        """
        if vbat is None:
            vbat = self.vbat_sag
        kvr = kv_rpm_per_v * 2.0 * math.pi / 60.0 * self.kv_load_factor
        n_free = kvr * vbat / (2.0 * math.pi)

        # Clamp RPM to prop table max to avoid extrapolation
        n_max_rps = min(n_free, prop.n_tab_max())
        if n_max_rps <= 0.02 * n_free:
            # Motor cannot reach even 2% of no-load speed at this voltage
            return 0.0, 0.0, 0.0

        def residual(n_rps: float) -> float:
            i_a = self.motor_current(prop, kv_rpm_per_v, i0_a, n_rps, v_mps)
            return i_a * (rm_ohm + self.r_pack) + 2.0 * math.pi * n_rps / kvr - vbat

        n_rps = brentq(residual, 0.02 * n_free, n_max_rps)
        thrust_n = prop_thrust(prop, n_rps, v_mps)
        current_a = self.motor_current(prop, kv_rpm_per_v, i0_a, n_rps, v_mps)
        return n_rps, thrust_n, current_a

    def rpm_for_thrust(
        self,
        prop,
        kv_rpm_per_v: float,
        rm_ohm: float,
        i0_a: float,
        v_mps: float,
        target_thrust_n: float,
        n_min_rps: float,
        n_max_rps: float,
        vbat: float = None,
    ):
        """Find RPM that produces target_thrust_n at airspeed v_mps.

        Returns: (n_rps, current_a, throttle, p_batt_w) or None
        """
        if vbat is None:
            vbat = self.vbat_sag
        if n_min_rps >= n_max_rps:
            return None

        def thrust_residual(n_rps: float) -> float:
            return prop_thrust(prop, n_rps, v_mps) - target_thrust_n

        # Check if target thrust is bracketed
        if thrust_residual(n_max_rps) < 0 or thrust_residual(n_min_rps) > 0:
            return None

        n_rps = brentq(thrust_residual, n_min_rps, n_max_rps)

        # Electrical operating point at this RPM
        kvr = kv_rpm_per_v * 2.0 * math.pi / 60.0 * self.kv_load_factor
        current_a = self.motor_current(prop, kv_rpm_per_v, i0_a, n_rps, v_mps)
        vm = current_a * rm_ohm + 2.0 * math.pi * n_rps / kvr
        v_load = vbat - current_a * self.r_pack
        throttle = vm / v_load
        p_batt_w = vm * current_a / self.esc_eff

        return n_rps, current_a, throttle, p_batt_w

    def check_current(self, i_a: float, burst: bool = False) -> tuple[bool, str]:
        """Check if current is within limits.

        Returns: (within_limit, message)
        """
        if burst:
            limit = self.i_lim_burst
            limit_name = "burst"
        else:
            limit = self.i_lim_cont
            limit_name = "continuous"

        within = i_a <= limit
        return within, f"{i_a:.1f}A vs {limit:.1f}A ({limit_name})"


def create_electrical_system(**kwargs) -> ElectricalSystem:
    """Factory function for easy creation."""
    return ElectricalSystem(**kwargs)