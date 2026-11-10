"""Longitudinal small-disturbance state-space construction."""

from __future__ import annotations

import numpy as np

from .aircraft_parameters import Aircraft
from .stability_derivatives import LongitudinalDerivatives
from .state_space import StateSpaceModel
from .trim import TrimCondition


def build_longitudinal_model(
    aircraft: Aircraft,
    trim: TrimCondition,
    derivatives: LongitudinalDerivatives,
) -> StateSpaceModel:
    """Build the [u, w, q, theta] model with [elevator, throttle] inputs."""

    denominator = 1.0 - derivatives.Zw_dot
    if abs(denominator) < 1e-10:
        raise ValueError("1 - Zw_dot is singular")

    z_row = np.array(
        [
            derivatives.Zu / denominator,
            derivatives.Zw / denominator,
            (trim.V0 + derivatives.Zq) / denominator,
            -aircraft.gravity * np.sin(trim.gamma0) / denominator,
        ]
    )
    x_row = np.array(
        [
            derivatives.Xu,
            derivatives.Xw,
            derivatives.Xq,
            -aircraft.gravity * np.cos(trim.gamma0),
        ]
    )
    moment_row = np.array(
        [
            derivatives.Mu + derivatives.Mw_dot * z_row[0],
            derivatives.Mw + derivatives.Mw_dot * z_row[1],
            derivatives.Mq + derivatives.Mw_dot * z_row[2],
            derivatives.Mw_dot * z_row[3],
        ]
    )
    A = np.vstack((x_row, z_row, moment_row, np.array([0.0, 0.0, 1.0, 0.0])))

    z_controls = np.array(
        [derivatives.Z_delta_e, derivatives.Z_delta_T]
    ) / denominator
    x_controls = np.array(
        [derivatives.X_delta_e, derivatives.X_delta_T]
    )
    moment_controls = np.array(
        [derivatives.M_delta_e, derivatives.M_delta_T]
    ) + derivatives.Mw_dot * z_controls
    B = np.vstack((x_controls, z_controls, moment_controls, np.zeros(2)))

    return StateSpaceModel(
        A=A,
        B=B,
        C=np.eye(4),
        D=np.zeros((4, 2)),
        state_names=("u", "w", "q", "theta"),
        input_names=("elevator", "throttle"),
        output_names=("u", "w", "q", "theta"),
        name="Longitudinal",
    )
