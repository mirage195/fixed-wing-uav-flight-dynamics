"""Strict TOML input for reproducible aircraft analyses."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, fields
from math import radians
from pathlib import Path
from typing import Any, TypeVar

import numpy as np

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - exercised on Python 3.10
    import tomli as tomllib

from .aerodynamic_data import (
    AerodynamicData,
    LateralCoefficientDerivatives,
    LongitudinalCoefficientDerivatives,
    get_example_aerodynamics,
)
from .aircraft_parameters import (
    Aircraft,
    AircraftTailData,
    ControlLimits,
    Fin,
    Fuselage,
    Inertia,
    MainWing,
    Propulsion,
    get_aircraft,
)

CONFIG_SCHEMA_VERSION = 1
DEFAULT_DATA_STATUS = "illustrative; not flight validated"
DerivativeType = TypeVar(
    "DerivativeType",
    LongitudinalCoefficientDerivatives,
    LateralCoefficientDerivatives,
)


class ConfigurationError(ValueError):
    """Raised when a configuration file is incomplete or ambiguous."""


@dataclass(frozen=True)
class AnalysisConfiguration:
    """Validated inputs and provenance for one analysis run."""

    aircraft: Aircraft
    aerodynamics: AerodynamicData
    data_status: str
    schema_version: int = CONFIG_SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != CONFIG_SCHEMA_VERSION:
            raise ConfigurationError(
                f"unsupported schema_version {self.schema_version}; "
                f"expected {CONFIG_SCHEMA_VERSION}"
            )
        if not self.data_status.strip():
            raise ConfigurationError("data_status cannot be empty")


def default_configuration() -> AnalysisConfiguration:
    """Return the bundled illustrative analysis configuration."""

    return AnalysisConfiguration(
        aircraft=get_aircraft(),
        aerodynamics=get_example_aerodynamics(),
        data_status=DEFAULT_DATA_STATUS,
    )


def _table(container: Mapping[str, Any], key: str, context: str) -> Mapping[str, Any]:
    value = container.get(key)
    if not isinstance(value, Mapping):
        raise ConfigurationError(f"{context}.{key} must be a TOML table")
    return value


def _validate_keys(
    table: Mapping[str, Any],
    context: str,
    required: set[str],
    optional: set[str] | None = None,
) -> None:
    optional = optional or set()
    missing = sorted(required - set(table))
    unknown = sorted(set(table) - required - optional)
    if missing:
        raise ConfigurationError(f"{context} is missing keys: {missing}")
    if unknown:
        raise ConfigurationError(f"{context} contains unknown keys: {unknown}")


def _text(table: Mapping[str, Any], key: str, context: str) -> str:
    value = table.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"{context}.{key} must be a non-empty string")
    return value


def _number(table: Mapping[str, Any], key: str, context: str) -> float:
    value = table.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{context}.{key} must be a number")
    number = float(value)
    if not np.isfinite(number):
        raise ConfigurationError(f"{context}.{key} must be finite")
    return number


def _numbers(
    table: Mapping[str, Any],
    key: str,
    context: str,
    *,
    length: int | None = None,
) -> tuple[float, ...]:
    value = table.get(key)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ConfigurationError(f"{context}.{key} must be an array of numbers")
    converted = tuple(
        _number({key: item}, key, f"{context}.{key}[{index}]")
        for index, item in enumerate(value)
    )
    if length is not None and len(converted) != length:
        raise ConfigurationError(
            f"{context}.{key} must contain exactly {length} values"
        )
    return converted


def _derivatives(
    table: Mapping[str, Any],
    context: str,
    derivative_class: type[DerivativeType],
) -> DerivativeType:
    names = {field.name for field in fields(derivative_class)}
    _validate_keys(table, context, names)
    return derivative_class(
        **{name: _number(table, name, context) for name in names}
    )


def _parse_aircraft(root: Mapping[str, Any]) -> Aircraft:
    context = "aircraft"
    required = {
        "name",
        "provenance",
        "mass_kg",
        "reference_speed_m_s",
        "air_density_kg_m3",
        "gravity_m_s2",
        "inertia",
        "wing",
        "fuselage",
        "fin",
        "tail",
        "propulsion",
        "controls",
    }
    optional = {"center_of_gravity_x_m", "aerodynamic_reference_x_m"}
    _validate_keys(root, context, required, optional)

    inertia = _table(root, "inertia", context)
    _validate_keys(inertia, "aircraft.inertia", {"Ixx", "Iyy", "Izz", "Ixz"})
    wing = _table(root, "wing", context)
    _validate_keys(
        wing,
        "aircraft.wing",
        {
            "area_m2",
            "span_m",
            "mean_chord_m",
            "sweep_quarter_chord_deg",
            "dihedral_deg",
        },
    )
    fuselage = _table(root, "fuselage", context)
    _validate_keys(
        fuselage,
        "aircraft.fuselage",
        {"side_area_m2", "aerodynamic_center_z_m"},
    )
    fin = _table(root, "fin", context)
    _validate_keys(
        fin,
        "aircraft.fin",
        {
            "area_m2",
            "moment_arm_m",
            "aerodynamic_center_z_m",
            "lift_curve_slope_per_rad",
        },
    )
    tail = _table(root, "tail", context)
    _validate_keys(
        tail,
        "aircraft.tail",
        {"area_m2", "moment_arm_m", "lift_curve_slope_per_rad"},
    )
    propulsion = _table(root, "propulsion", context)
    _validate_keys(
        propulsion,
        "aircraft.propulsion",
        {"maximum_thrust_n", "thrust_speed_derivative_n_s_m"},
    )
    controls = _table(root, "controls", context)
    _validate_keys(
        controls,
        "aircraft.controls",
        {"elevator_deg", "aileron_deg", "rudder_deg", "throttle"},
    )

    cg = (
        _number(root, "center_of_gravity_x_m", context)
        if "center_of_gravity_x_m" in root
        else None
    )
    reference = (
        _number(root, "aerodynamic_reference_x_m", context)
        if "aerodynamic_reference_x_m" in root
        else None
    )

    def angle_limits(key: str) -> tuple[float, float]:
        return tuple(
            radians(value)
            for value in _numbers(
                controls,
                key,
                "aircraft.controls",
                length=2,
            )
        )

    return Aircraft(
        name=_text(root, "name", context),
        provenance=_text(root, "provenance", context),
        mass=_number(root, "mass_kg", context),
        inertia=Inertia(
            Ixx=_number(inertia, "Ixx", "aircraft.inertia"),
            Iyy=_number(inertia, "Iyy", "aircraft.inertia"),
            Izz=_number(inertia, "Izz", "aircraft.inertia"),
            Ixz=_number(inertia, "Ixz", "aircraft.inertia"),
        ),
        wing=MainWing(
            area=_number(wing, "area_m2", "aircraft.wing"),
            span=_number(wing, "span_m", "aircraft.wing"),
            mean_chord=_number(wing, "mean_chord_m", "aircraft.wing"),
            sweep_quarter_chord=radians(
                _number(wing, "sweep_quarter_chord_deg", "aircraft.wing")
            ),
            dihedral=radians(_number(wing, "dihedral_deg", "aircraft.wing")),
        ),
        fuselage=Fuselage(
            side_area=_number(fuselage, "side_area_m2", "aircraft.fuselage"),
            aerodynamic_center_z=_number(
                fuselage,
                "aerodynamic_center_z_m",
                "aircraft.fuselage",
            ),
        ),
        fin=Fin(
            area=_number(fin, "area_m2", "aircraft.fin"),
            moment_arm=_number(fin, "moment_arm_m", "aircraft.fin"),
            aerodynamic_center_z=_number(
                fin,
                "aerodynamic_center_z_m",
                "aircraft.fin",
            ),
            lift_curve_slope=_number(
                fin,
                "lift_curve_slope_per_rad",
                "aircraft.fin",
            ),
        ),
        tail=AircraftTailData(
            area=_number(tail, "area_m2", "aircraft.tail"),
            moment_arm=_number(tail, "moment_arm_m", "aircraft.tail"),
            lift_curve_slope=_number(
                tail,
                "lift_curve_slope_per_rad",
                "aircraft.tail",
            ),
        ),
        propulsion=Propulsion(
            maximum_thrust=_number(
                propulsion,
                "maximum_thrust_n",
                "aircraft.propulsion",
            ),
            thrust_speed_derivative=_number(
                propulsion,
                "thrust_speed_derivative_n_s_m",
                "aircraft.propulsion",
            ),
        ),
        controls=ControlLimits(
            elevator=angle_limits("elevator_deg"),
            aileron=angle_limits("aileron_deg"),
            rudder=angle_limits("rudder_deg"),
            throttle=_numbers(
                controls,
                "throttle",
                "aircraft.controls",
                length=2,
            ),
        ),
        reference_speed=_number(root, "reference_speed_m_s", context),
        center_of_gravity_x=cg,
        aerodynamic_reference_x=reference,
        air_density=_number(root, "air_density_kg_m3", context),
        gravity=_number(root, "gravity_m_s2", context),
    )


def _parse_aerodynamics(root: Mapping[str, Any]) -> AerodynamicData:
    context = "aerodynamics"
    required = {
        "provenance",
        "speed_range_m_s",
        "alpha_deg",
        "CL",
        "CD",
        "Cm",
        "longitudinal_derivatives",
        "lateral_derivatives",
    }
    _validate_keys(root, context, required)
    longitudinal = _table(root, "longitudinal_derivatives", context)
    lateral = _table(root, "lateral_derivatives", context)

    return AerodynamicData(
        provenance=_text(root, "provenance", context),
        speed_range=_numbers(
            root,
            "speed_range_m_s",
            context,
            length=2,
        ),
        alpha=np.deg2rad(_numbers(root, "alpha_deg", context)),
        CL=np.asarray(_numbers(root, "CL", context)),
        CD=np.asarray(_numbers(root, "CD", context)),
        Cm=np.asarray(_numbers(root, "Cm", context)),
        longitudinal=_derivatives(
            longitudinal,
            "aerodynamics.longitudinal_derivatives",
            LongitudinalCoefficientDerivatives,
        ),
        lateral=_derivatives(
            lateral,
            "aerodynamics.lateral_derivatives",
            LateralCoefficientDerivatives,
        ),
    )


def configuration_from_mapping(root: Mapping[str, Any]) -> AnalysisConfiguration:
    """Validate and construct configuration objects from a parsed TOML mapping."""

    required = {"schema_version", "data_status", "aircraft", "aerodynamics"}
    _validate_keys(root, "configuration", required)
    schema_version = root.get("schema_version")
    if isinstance(schema_version, bool) or not isinstance(schema_version, int):
        raise ConfigurationError("schema_version must be an integer")
    if schema_version != CONFIG_SCHEMA_VERSION:
        raise ConfigurationError(
            f"unsupported schema_version {schema_version}; "
            f"expected {CONFIG_SCHEMA_VERSION}"
        )
    return AnalysisConfiguration(
        aircraft=_parse_aircraft(_table(root, "aircraft", "configuration")),
        aerodynamics=_parse_aerodynamics(
            _table(root, "aerodynamics", "configuration")
        ),
        data_status=_text(root, "data_status", "configuration"),
        schema_version=schema_version,
    )


def load_configuration(path: str | Path) -> AnalysisConfiguration:
    """Load a UTF-8 TOML file and reject missing or unknown fields."""

    source = Path(path)
    try:
        with source.open("rb") as stream:
            parsed = tomllib.load(stream)
    except tomllib.TOMLDecodeError as error:
        raise ConfigurationError(f"invalid TOML in {source}: {error}") from error
    return configuration_from_mapping(parsed)
