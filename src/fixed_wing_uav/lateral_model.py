"""Lateral-directional small-disturbance state-space construction."""

from __future__ import annotations

import numpy as np

from .aircraft_parameters import Aircraft
from .lateral_derivatives import LateralDerivatives
from .state_space import StateSpaceModel
from .trim import TrimCondition


def build_lateral_model(
    aircraft: Aircraft,
    trim: TrimCondition,
    derivatives: LateralDerivatives,
) -> StateSpaceModel:
    """Build the [v, p, r, phi] model with [aileron, rudder] inputs.

    The supplied L/N derivatives already include roll-yaw cross-inertia
    coupling. No second division by Ixx or Izz is performed here.
    """

    A = np.array(
        [
            [
                derivatives.Yv,
                derivatives.Yp,
                derivatives.Yr - trim.V0,
                aircraft.gravity * np.cos(trim.gamma0),
            ],
            [derivatives.Lv, derivatives.Lp, derivatives.Lr, 0.0],
            [derivatives.Nv, derivatives.Np, derivatives.Nr, 0.0],
            [0.0, 1.0, np.tan(trim.gamma0), 0.0],
        ]
    )
    B = np.array(
        [
            [derivatives.Y_delta_a, derivatives.Y_delta_r],
            [derivatives.L_delta_a, derivatives.L_delta_r],
            [derivatives.N_delta_a, derivatives.N_delta_r],
            [0.0, 0.0],
        ]
    )

    return StateSpaceModel(
        A=A,
        B=B,
        C=np.eye(4),
        D=np.zeros((4, 2)),
        state_names=("v", "p", "r", "phi"),
        input_names=("aileron", "rudder"),
        output_names=("v", "p", "r", "phi"),
        name="Lateral-directional",
    )
