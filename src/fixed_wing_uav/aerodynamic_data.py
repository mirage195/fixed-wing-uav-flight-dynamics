"""Validated aerodynamic coefficient data.

All angular derivatives use radians. Rotary derivatives use the conventional
non-dimensional rates p*b/(2*V), q*c/(2*V), and r*b/(2*V).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


def _validated_curve(name: str, values: FloatArray, size: int) -> FloatArray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1 or array.size != size:
        raise ValueError(f"{name} must be a one-dimensional array of length {size}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains non-finite values")
    array = array.copy()
    array.setflags(write=False)
    return array


def _validate_derivative_values(instance: object) -> None:
    values = np.fromiter(asdict(instance).values(), dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("aerodynamic derivatives must all be finite")


@dataclass(frozen=True)
class LongitudinalCoefficientDerivatives:
    """Non-dimensional longitudinal rate and control derivatives."""

    CL_q: float
    Cm_q: float
    CL_alpha_dot: float
    Cm_alpha_dot: float
    CL_delta_e: float
    CD_delta_e: float
    Cm_delta_e: float
    CX_q: float = 0.0
    dCL_dV: float = 0.0
    dCD_dV: float = 0.0
    dCm_dV: float = 0.0

    def __post_init__(self) -> None:
        _validate_derivative_values(self)


@dataclass(frozen=True)
class LateralCoefficientDerivatives:
    """Non-dimensional lateral-directional stability/control derivatives."""

    CY_beta: float
    Cl_beta: float
    Cn_beta: float
    CY_p: float
    Cl_p: float
    Cn_p: float
    CY_r: float
    Cl_r: float
    Cn_r: float
    CY_delta_a: float
    Cl_delta_a: float
    Cn_delta_a: float
    CY_delta_r: float
    Cl_delta_r: float
    Cn_delta_r: float

    def __post_init__(self) -> None:
        _validate_derivative_values(self)


@dataclass(frozen=True)
class AerodynamicData:
    """Aerodynamic lookup curves plus derivatives at the analysis condition."""

    provenance: str
    speed_range: tuple[float, float]
    alpha: FloatArray
    CL: FloatArray
    CD: FloatArray
    Cm: FloatArray
    longitudinal: LongitudinalCoefficientDerivatives
    lateral: LateralCoefficientDerivatives

    def __post_init__(self) -> None:
        speed_range = tuple(float(value) for value in self.speed_range)
        if (
            len(speed_range) != 2
            or not np.all(np.isfinite(speed_range))
            or speed_range[0] <= 0.0
            or speed_range[0] >= speed_range[1]
        ):
            raise ValueError("speed_range must be finite, positive, and increasing")
        object.__setattr__(self, "speed_range", speed_range)
        alpha = np.asarray(self.alpha, dtype=float)
        if alpha.ndim != 1 or alpha.size < 3:
            raise ValueError("alpha must contain at least three one-dimensional samples")
        if not np.all(np.isfinite(alpha)) or not np.all(np.diff(alpha) > 0.0):
            raise ValueError("alpha samples must be finite and strictly increasing")
        alpha = alpha.copy()
        alpha.setflags(write=False)
        object.__setattr__(self, "alpha", alpha)
        object.__setattr__(self, "CL", _validated_curve("CL", self.CL, alpha.size))
        object.__setattr__(self, "CD", _validated_curve("CD", self.CD, alpha.size))
        object.__setattr__(self, "Cm", _validated_curve("Cm", self.Cm, alpha.size))
        if np.any(self.CD < 0.0):
            raise ValueError("CD values cannot be negative")
        if not self.provenance.strip():
            raise ValueError("aerodynamic-data provenance cannot be empty")

    def validate_speed(self, speed: float) -> None:
        if not self.speed_range[0] <= speed <= self.speed_range[1]:
            raise ValueError(
                f"speed {speed:.3f} m/s is outside the aerodynamic-data range "
                f"[{self.speed_range[0]:.3f}, {self.speed_range[1]:.3f}] m/s"
            )

    def _interpolate(self, values: FloatArray, alpha: float) -> float:
        if not self.alpha[0] <= alpha <= self.alpha[-1]:
            raise ValueError(
                f"angle of attack {np.degrees(alpha):.3f} deg is outside the "
                f"data range [{np.degrees(self.alpha[0]):.3f}, "
                f"{np.degrees(self.alpha[-1]):.3f}] deg"
            )
        return float(np.interp(alpha, self.alpha, values))

    def coefficients(self, alpha: float, elevator: float = 0.0) -> tuple[float, float, float]:
        """Return CL, CD, and Cm without extrapolating the supplied curves."""

        long = self.longitudinal
        CL = self._interpolate(self.CL, alpha) + long.CL_delta_e * elevator
        CD = self._interpolate(self.CD, alpha) + long.CD_delta_e * elevator
        Cm = self._interpolate(self.Cm, alpha) + long.Cm_delta_e * elevator
        if CD < 0.0:
            raise ValueError("interpolated drag coefficient is negative")
        return CL, CD, Cm

    def alpha_slopes(self, alpha: float) -> tuple[float, float, float]:
        """Return local piecewise-linear dCL/dalpha, dCD/dalpha, dCm/dalpha."""

        if not self.alpha[0] <= alpha <= self.alpha[-1]:
            self._interpolate(self.CL, alpha)  # raises the detailed range error
        index = int(np.searchsorted(self.alpha, alpha, side="right") - 1)
        index = min(max(index, 0), self.alpha.size - 2)
        delta_alpha = self.alpha[index + 1] - self.alpha[index]
        return tuple(
            float((curve[index + 1] - curve[index]) / delta_alpha)
            for curve in (self.CL, self.CD, self.Cm)
        )


def get_example_aerodynamics() -> AerodynamicData:
    """Return illustrative coefficients inherited from the prototype.

    Control derivatives were added only to make the complete state-space
    interface executable. They must be replaced before aircraft-specific use.
    """

    return AerodynamicData(
        provenance=(
            "Illustrative curves based on the original repository and provisional "
            "control derivatives; not wind-tunnel, CFD, or flight-test validated."
        ),
        speed_range=(20.0, 30.0),
        alpha=np.deg2rad(np.array([-4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0])),
        CL=np.array([-0.05, 0.10, 0.25, 0.40, 0.55, 0.70, 0.85, 0.95]),
        CD=np.array([0.035, 0.031, 0.030, 0.031, 0.034, 0.040, 0.050, 0.065]),
        Cm=np.array([0.060, 0.040, 0.020, 0.000, -0.020, -0.040, -0.060, -0.080]),
        longitudinal=LongitudinalCoefficientDerivatives(
            CL_q=7.0,
            Cm_q=-18.0,
            CL_alpha_dot=2.5,
            Cm_alpha_dot=-7.0,
            CL_delta_e=0.30,
            CD_delta_e=0.0,
            Cm_delta_e=-1.10,
        ),
        lateral=LateralCoefficientDerivatives(
            CY_beta=-0.4618,
            Cl_beta=-0.0157,
            Cn_beta=0.1254,
            CY_p=-0.5486,
            Cl_p=-0.4832,
            Cn_p=-0.0418,
            CY_r=0.1254,
            Cl_r=0.1046,
            Cn_r=-0.0480,
            CY_delta_a=0.0,
            Cl_delta_a=0.12,
            Cn_delta_a=0.02,
            CY_delta_r=0.18,
            Cl_delta_r=0.02,
            Cn_delta_r=-0.10,
        ),
    )
