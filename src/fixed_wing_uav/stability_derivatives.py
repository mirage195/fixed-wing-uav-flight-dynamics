"""Conversion of longitudinal aerodynamic coefficients to SI derivatives."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from .aerodynamic_data import AerodynamicData
from .aircraft_parameters import Aircraft
from .trim import TrimCondition


@dataclass(frozen=True)
class LongitudinalDerivatives:
    """Dimensional acceleration derivatives for [u, w, q, theta].

    Force derivatives have already been divided by mass and moment derivatives
    by pitch inertia. They therefore enter the state matrices directly.
    """

    Xu: float
    Xw: float
    Xq: float
    Zu: float
    Zw: float
    Zq: float
    Mu: float
    Mw: float
    Mq: float
    Zw_dot: float
    Mw_dot: float
    X_delta_e: float
    Z_delta_e: float
    M_delta_e: float
    X_delta_T: float
    Z_delta_T: float = 0.0
    M_delta_T: float = 0.0

    def __post_init__(self) -> None:
        values = np.fromiter(asdict(self).values(), dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("longitudinal derivatives must all be finite")

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


def compute_all_derivatives(
    aircraft: Aircraft,
    trim: TrimCondition,
    aero_data: AerodynamicData,
) -> LongitudinalDerivatives:
    """Return dimensional longitudinal derivatives at the trim condition.

    Coefficient-rate conventions are documented in docs/conventions.md.
    """

    V0 = trim.V0
    qbar = trim.dynamic_pressure
    S = aircraft.S
    c = aircraft.c
    mass = aircraft.mass
    Iyy = aircraft.Iy
    long = aero_data.longitudinal
    CL_alpha, CD_alpha, Cm_alpha = aero_data.alpha_slopes(trim.alpha0)

    force_velocity_scale = qbar * S / (mass * V0)
    moment_velocity_scale = qbar * S * c / (Iyy * V0)

    Xu = force_velocity_scale * (-2.0 * trim.CD0 - V0 * long.dCD_dV)
    Xu += aircraft.propulsion.thrust_speed_derivative / mass
    Zu = force_velocity_scale * (-2.0 * trim.CL0 - V0 * long.dCL_dV)
    Xw = force_velocity_scale * (trim.CL0 - CD_alpha)
    Zw = force_velocity_scale * (-(CL_alpha + trim.CD0))

    Xq = qbar * S / mass * long.CX_q * c / (2.0 * V0)
    Zq = -qbar * S / mass * long.CL_q * c / (2.0 * V0)

    Mu = moment_velocity_scale * (2.0 * trim.Cm0 + V0 * long.dCm_dV)
    Mw = moment_velocity_scale * Cm_alpha
    Mq = qbar * S * c / Iyy * long.Cm_q * c / (2.0 * V0)

    Zw_dot = -trim.rho * S * c * long.CL_alpha_dot / (4.0 * mass)
    Mw_dot = trim.rho * S * c**2 * long.Cm_alpha_dot / (4.0 * Iyy)

    # Stability-axis X opposes drag and Z opposes lift.
    X_delta_e = -qbar * S / mass * long.CD_delta_e
    Z_delta_e = -qbar * S / mass * long.CL_delta_e
    M_delta_e = qbar * S * c / Iyy * long.Cm_delta_e

    return LongitudinalDerivatives(
        Xu=Xu,
        Xw=Xw,
        Xq=Xq,
        Zu=Zu,
        Zw=Zw,
        Zq=Zq,
        Mu=Mu,
        Mw=Mw,
        Mq=Mq,
        Zw_dot=Zw_dot,
        Mw_dot=Mw_dot,
        X_delta_e=X_delta_e,
        Z_delta_e=Z_delta_e,
        M_delta_e=M_delta_e,
        X_delta_T=aircraft.propulsion.maximum_thrust / mass,
    )
