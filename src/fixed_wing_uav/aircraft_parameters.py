"""Validated aircraft and flight-condition parameter models.

The values returned by :func:`get_aircraft` are an illustrative example. They
are not a certified or flight-tested aircraft data set.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, radians


def _positive(name: str, value: float) -> None:
    if not isfinite(value) or not value > 0.0:
        raise ValueError(f"{name} must be positive; received {value!r}")


def _finite(name: str, value: float) -> None:
    if not isfinite(value):
        raise ValueError(f"{name} must be finite; received {value!r}")


@dataclass(frozen=True)
class Inertia:
    """Aircraft inertia terms about the analysis-axis center of gravity."""

    Ixx: float
    Iyy: float
    Izz: float
    Ixz: float = 0.0

    def __post_init__(self) -> None:
        _positive("Ixx", self.Ixx)
        _positive("Iyy", self.Iyy)
        _positive("Izz", self.Izz)
        if not isfinite(self.Ixz):
            raise ValueError("Ixz must be finite")
        if self.determinant_xz <= 0.0:
            raise ValueError("Ixx * Izz - Ixz**2 must be positive")

    @property
    def determinant_xz(self) -> float:
        return self.Ixx * self.Izz - self.Ixz**2


@dataclass(frozen=True)
class MainWing:
    area: float
    span: float
    mean_chord: float
    sweep_quarter_chord: float = 0.0
    dihedral: float = 0.0

    def __post_init__(self) -> None:
        _positive("wing area", self.area)
        _positive("wing span", self.span)
        _positive("wing mean chord", self.mean_chord)
        _finite("wing quarter-chord sweep", self.sweep_quarter_chord)
        _finite("wing dihedral", self.dihedral)

    @property
    def aspect_ratio(self) -> float:
        return self.span**2 / self.area


@dataclass(frozen=True)
class Fuselage:
    side_area: float
    aerodynamic_center_z: float

    def __post_init__(self) -> None:
        _positive("fuselage side area", self.side_area)
        _finite("fuselage aerodynamic-center z", self.aerodynamic_center_z)


@dataclass(frozen=True)
class Fin:
    area: float
    moment_arm: float
    aerodynamic_center_z: float
    lift_curve_slope: float

    def __post_init__(self) -> None:
        _positive("fin area", self.area)
        _positive("fin moment arm", self.moment_arm)
        _positive("fin lift-curve slope", self.lift_curve_slope)
        _finite("fin aerodynamic-center z", self.aerodynamic_center_z)


@dataclass(frozen=True)
class AircraftTailData:
    area: float
    moment_arm: float
    lift_curve_slope: float

    def __post_init__(self) -> None:
        _positive("horizontal-tail area", self.area)
        _positive("horizontal-tail moment arm", self.moment_arm)
        _positive("horizontal-tail lift-curve slope", self.lift_curve_slope)


@dataclass(frozen=True)
class Propulsion:
    """Simple normalized-throttle propulsion model used by the trim solver."""

    maximum_thrust: float
    thrust_speed_derivative: float = 0.0

    def __post_init__(self) -> None:
        _positive("maximum thrust", self.maximum_thrust)
        _finite("thrust speed derivative", self.thrust_speed_derivative)


@dataclass(frozen=True)
class ControlLimits:
    elevator: tuple[float, float]
    aileron: tuple[float, float]
    rudder: tuple[float, float]
    throttle: tuple[float, float] = (0.0, 1.0)

    def __post_init__(self) -> None:
        for name, bounds in (
            ("elevator", self.elevator),
            ("aileron", self.aileron),
            ("rudder", self.rudder),
            ("throttle", self.throttle),
        ):
            if (
                len(bounds) != 2
                or not all(isfinite(value) for value in bounds)
                or bounds[0] >= bounds[1]
            ):
                raise ValueError(f"invalid {name} limits: {bounds!r}")
        if self.throttle[0] < 0.0 or self.throttle[1] > 1.0:
            raise ValueError("normalized throttle limits must lie in [0, 1]")


@dataclass(frozen=True)
class Aircraft:
    name: str
    provenance: str
    mass: float
    inertia: Inertia
    wing: MainWing
    fuselage: Fuselage
    fin: Fin
    tail: AircraftTailData
    propulsion: Propulsion
    controls: ControlLimits
    reference_speed: float
    center_of_gravity_x: float | None = None
    aerodynamic_reference_x: float | None = None
    air_density: float = 1.225
    gravity: float = 9.80665

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("aircraft name cannot be empty")
        if not self.provenance.strip():
            raise ValueError("aircraft provenance cannot be empty")
        _positive("mass", self.mass)
        _positive("reference speed", self.reference_speed)
        _positive("air density", self.air_density)
        _positive("gravity", self.gravity)
        cg_and_reference = (self.center_of_gravity_x, self.aerodynamic_reference_x)
        if (cg_and_reference[0] is None) != (cg_and_reference[1] is None):
            raise ValueError(
                "center_of_gravity_x and aerodynamic_reference_x must be supplied together"
            )
        for name, value in zip(
            ("center_of_gravity_x", "aerodynamic_reference_x"),
            cg_and_reference,
            strict=True,
        ):
            if value is not None:
                _finite(name, value)

    # Compatibility/readability properties for conventional symbols.
    @property
    def S(self) -> float:
        return self.wing.area

    @property
    def b(self) -> float:
        return self.wing.span

    @property
    def c(self) -> float:
        return self.wing.mean_chord

    @property
    def U0(self) -> float:
        return self.reference_speed

    @property
    def rho(self) -> float:
        return self.air_density

    @property
    def Ix(self) -> float:
        return self.inertia.Ixx

    @property
    def Iy(self) -> float:
        return self.inertia.Iyy

    @property
    def Iz(self) -> float:
        return self.inertia.Izz

    @property
    def Ixz(self) -> float:
        return self.inertia.Ixz

    @property
    def static_margin(self) -> float | None:
        if self.center_of_gravity_x is None or self.aerodynamic_reference_x is None:
            return None
        return (self.aerodynamic_reference_x - self.center_of_gravity_x) / self.c


def get_aircraft() -> Aircraft:
    """Return the repository's illustrative, non-flight-validated UAV."""

    return Aircraft(
        name="Illustrative 13.5 kg fixed-wing UAV",
        provenance=(
            "Example values inherited from the original repository; aerodynamic "
            "and flight-test provenance has not yet been supplied."
        ),
        mass=13.5,
        inertia=Inertia(Ixx=0.8244, Iyy=1.135, Izz=1.759, Ixz=0.1204),
        wing=MainWing(
            area=0.55,
            span=2.90,
            mean_chord=0.19,
            sweep_quarter_chord=0.0,
            dihedral=0.0,
        ),
        fuselage=Fuselage(side_area=0.20, aerodynamic_center_z=-0.02),
        fin=Fin(
            area=0.10,
            moment_arm=0.80,
            aerodynamic_center_z=0.10,
            lift_curve_slope=2.5,
        ),
        tail=AircraftTailData(
            area=0.12,
            moment_arm=0.90,
            lift_curve_slope=3.5,
        ),
        propulsion=Propulsion(maximum_thrust=30.0),
        controls=ControlLimits(
            elevator=(-radians(25.0), radians(25.0)),
            aileron=(-radians(20.0), radians(20.0)),
            rudder=(-radians(25.0), radians(25.0)),
        ),
        reference_speed=25.0,
    )
