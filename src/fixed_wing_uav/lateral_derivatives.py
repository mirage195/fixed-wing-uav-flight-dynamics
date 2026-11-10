"""Conversion of lateral aerodynamic coefficients to SI derivatives."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from .aerodynamic_data import AerodynamicData
from .aircraft_parameters import Aircraft
from .trim import TrimCondition


@dataclass(frozen=True)
class LateralDerivatives:
    """Dimensional acceleration derivatives for [v, p, r, phi].

    L and N entries include the Ixz cross-inertia correction and therefore
    enter the state matrices directly.
    """

    Yv: float
    Yp: float
    Yr: float
    Lv: float
    Lp: float
    Lr: float
    Nv: float
    Np: float
    Nr: float
    Y_delta_a: float
    Y_delta_r: float
    L_delta_a: float
    L_delta_r: float
    N_delta_a: float
    N_delta_r: float

    def __post_init__(self) -> None:
        values = np.fromiter(asdict(self).values(), dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("lateral derivatives must all be finite")

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


def _coupled_angular_derivatives(
    aircraft: Aircraft,
    rolling_moment_derivative: float,
    yawing_moment_derivative: float,
) -> tuple[float, float]:
    """Solve the coupled roll/yaw inertia equations for p_dot and r_dot."""

    inertia = aircraft.inertia
    gamma = inertia.determinant_xz
    roll_acceleration = (
        inertia.Izz * rolling_moment_derivative
        + inertia.Ixz * yawing_moment_derivative
    ) / gamma
    yaw_acceleration = (
        inertia.Ixz * rolling_moment_derivative
        + inertia.Ixx * yawing_moment_derivative
    ) / gamma
    return roll_acceleration, yaw_acceleration


def compute_lateral_derivatives(
    aircraft: Aircraft,
    aero_data: AerodynamicData,
    trim: TrimCondition,
) -> LateralDerivatives:
    """Return dimensional lateral derivatives at the trim condition."""

    lat = aero_data.lateral
    V0 = trim.V0
    qbar = trim.dynamic_pressure
    S = aircraft.S
    b = aircraft.b
    mass = aircraft.mass

    Yv = qbar * S / (mass * V0) * lat.CY_beta
    Yp = qbar * S / mass * lat.CY_p * b / (2.0 * V0)
    Yr = qbar * S / mass * lat.CY_r * b / (2.0 * V0)

    moment_beta_scale = qbar * S * b / V0
    moment_rate_scale = qbar * S * b**2 / (2.0 * V0)
    moment_control_scale = qbar * S * b

    Lv, Nv = _coupled_angular_derivatives(
        aircraft,
        moment_beta_scale * lat.Cl_beta,
        moment_beta_scale * lat.Cn_beta,
    )
    Lp, Np = _coupled_angular_derivatives(
        aircraft,
        moment_rate_scale * lat.Cl_p,
        moment_rate_scale * lat.Cn_p,
    )
    Lr, Nr = _coupled_angular_derivatives(
        aircraft,
        moment_rate_scale * lat.Cl_r,
        moment_rate_scale * lat.Cn_r,
    )
    L_delta_a, N_delta_a = _coupled_angular_derivatives(
        aircraft,
        moment_control_scale * lat.Cl_delta_a,
        moment_control_scale * lat.Cn_delta_a,
    )
    L_delta_r, N_delta_r = _coupled_angular_derivatives(
        aircraft,
        moment_control_scale * lat.Cl_delta_r,
        moment_control_scale * lat.Cn_delta_r,
    )

    return LateralDerivatives(
        Yv=Yv,
        Yp=Yp,
        Yr=Yr,
        Lv=Lv,
        Lp=Lp,
        Lr=Lr,
        Nv=Nv,
        Np=Np,
        Nr=Nr,
        Y_delta_a=qbar * S / mass * lat.CY_delta_a,
        Y_delta_r=qbar * S / mass * lat.CY_delta_r,
        L_delta_a=L_delta_a,
        L_delta_r=L_delta_r,
        N_delta_a=N_delta_a,
        N_delta_r=N_delta_r,
    )
