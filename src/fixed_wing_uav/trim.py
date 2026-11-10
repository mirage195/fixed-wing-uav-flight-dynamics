"""Bounded steady-flight trim calculations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares

from .aerodynamic_data import AerodynamicData
from .aircraft_parameters import Aircraft


@dataclass(frozen=True)
class TrimCondition:
    """Straight-flight equilibrium and its numerical verification data."""

    V0: float
    alpha0: float
    gamma0: float
    pitch0: float
    elevator0: float
    throttle0: float
    thrust0: float
    CL0: float
    CD0: float
    Cm0: float
    rho: float
    S: float
    c_bar: float
    residual_forces: tuple[float, float]
    residual_pitch_moment: float
    solver_message: str

    def __post_init__(self) -> None:
        values = np.array(
            [
                self.V0,
                self.alpha0,
                self.gamma0,
                self.pitch0,
                self.elevator0,
                self.throttle0,
                self.thrust0,
                self.CL0,
                self.CD0,
                self.Cm0,
                self.rho,
                self.S,
                self.c_bar,
                *self.residual_forces,
                self.residual_pitch_moment,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("trim values and residuals must all be finite")
        if self.V0 <= 0.0 or self.rho <= 0.0 or self.S <= 0.0 or self.c_bar <= 0.0:
            raise ValueError("trim speed, density, area, and chord must be positive")
        if not 0.0 <= self.throttle0 <= 1.0:
            raise ValueError("trim throttle must lie in [0, 1]")
        if self.thrust0 < 0.0 or self.CD0 < 0.0:
            raise ValueError("trim thrust and drag coefficient cannot be negative")
        if len(self.residual_forces) != 2:
            raise ValueError("residual_forces must contain lift and axial residuals")
        if not self.solver_message:
            raise ValueError("solver_message cannot be empty")

    @property
    def dynamic_pressure(self) -> float:
        return 0.5 * self.rho * self.V0**2

    @property
    def Sc(self) -> float:
        return self.S * self.c_bar

    @property
    def maximum_normalized_residual(self) -> float:
        force_scale = self.dynamic_pressure * self.S
        moment_scale = force_scale * self.c_bar
        return max(
            abs(self.residual_forces[0]) / force_scale,
            abs(self.residual_forces[1]) / force_scale,
            abs(self.residual_pitch_moment) / moment_scale,
        )


def solve_level_trim(
    aircraft: Aircraft,
    aerodynamics: AerodynamicData,
    *,
    speed: float | None = None,
    flight_path_angle: float = 0.0,
    residual_tolerance: float = 1e-8,
) -> TrimCondition:
    """Solve lift, pitch-moment, and axial-force equilibrium.

    The independent variables are angle of attack, elevator deflection, and
    normalized throttle. The speed and flight-path angle are prescribed.
    """

    V0 = aircraft.reference_speed if speed is None else float(speed)
    if not np.isfinite(V0) or not V0 > 0.0:
        raise ValueError("trim speed must be positive")
    aerodynamics.validate_speed(V0)
    if not np.isfinite(flight_path_angle):
        raise ValueError("flight_path_angle must be finite")
    if residual_tolerance <= 0.0:
        raise ValueError("residual_tolerance must be positive")

    rho = aircraft.air_density
    qbar = 0.5 * rho * V0**2
    weight = aircraft.mass * aircraft.gravity
    force_scale = weight
    moment_scale = weight * aircraft.c
    elevator_limits = aircraft.controls.elevator
    throttle_limits = aircraft.controls.throttle

    def physical_residuals(x: NDArray[np.float64]) -> NDArray[np.float64]:
        alpha, elevator, throttle = x
        CL, CD, Cm = aerodynamics.coefficients(alpha, elevator)
        lift = qbar * aircraft.S * CL
        drag = qbar * aircraft.S * CD
        pitching_moment = qbar * aircraft.S * aircraft.c * Cm
        thrust = throttle * aircraft.propulsion.maximum_thrust
        return np.array(
            [
                lift - weight * np.cos(flight_path_angle),
                thrust - drag - weight * np.sin(flight_path_angle),
                pitching_moment,
            ],
            dtype=float,
        )

    def scaled_residuals(x: NDArray[np.float64]) -> NDArray[np.float64]:
        residuals = physical_residuals(x)
        return residuals / np.array([force_scale, force_scale, moment_scale])

    alpha_guess = float(np.clip(np.deg2rad(5.0), aerodynamics.alpha[0], aerodynamics.alpha[-1]))
    _, drag_guess, _ = aerodynamics.coefficients(alpha_guess)
    throttle_guess = np.clip(
        qbar * aircraft.S * drag_guess / aircraft.propulsion.maximum_thrust,
        *throttle_limits,
    )
    initial_guess = np.array([alpha_guess, 0.0, throttle_guess], dtype=float)
    lower_bounds = np.array(
        [aerodynamics.alpha[0], elevator_limits[0], throttle_limits[0]], dtype=float
    )
    upper_bounds = np.array(
        [aerodynamics.alpha[-1], elevator_limits[1], throttle_limits[1]], dtype=float
    )

    solution = least_squares(
        scaled_residuals,
        initial_guess,
        bounds=(lower_bounds, upper_bounds),
        xtol=1e-13,
        ftol=1e-13,
        gtol=1e-13,
        max_nfev=1000,
    )
    physical = physical_residuals(solution.x)
    max_residual = float(np.max(np.abs(scaled_residuals(solution.x))))
    if not solution.success or max_residual > residual_tolerance:
        raise RuntimeError(
            "trim did not converge to the requested tolerance: "
            f"success={solution.success}, maximum normalized residual={max_residual:.3e}, "
            f"message={solution.message}"
        )

    alpha0, elevator0, throttle0 = (float(value) for value in solution.x)
    CL0, CD0, Cm0 = aerodynamics.coefficients(alpha0, elevator0)
    return TrimCondition(
        V0=V0,
        alpha0=alpha0,
        gamma0=flight_path_angle,
        pitch0=alpha0 + flight_path_angle,
        elevator0=elevator0,
        throttle0=throttle0,
        thrust0=throttle0 * aircraft.propulsion.maximum_thrust,
        CL0=CL0,
        CD0=CD0,
        Cm0=Cm0,
        rho=rho,
        S=aircraft.S,
        c_bar=aircraft.c,
        residual_forces=(float(physical[0]), float(physical[1])),
        residual_pitch_moment=float(physical[2]),
        solver_message=str(solution.message),
    )
