"""Structured eigenvalue, modal-metric, and mode-shape analysis."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from math import log, pi, sqrt

import numpy as np
from numpy.typing import NDArray

from .state_space import StateSpaceModel

ComplexArray = NDArray[np.complex128]
FloatArray = NDArray[np.float64]
_ROOT_TOLERANCE = 1e-8


def _complex_dict(value: complex) -> dict[str, float]:
    return {"real": float(value.real), "imag": float(value.imag)}


@dataclass(frozen=True)
class ModeResult:
    """Metrics and state participation for one physical dynamic mode."""

    identifier: str
    name: str
    eigenvalues: tuple[complex, ...]
    stability: str
    response_type: str
    damping_character: str
    natural_frequency_rad_s: float | None
    damped_frequency_rad_s: float
    damping_ratio: float | None
    period_s: float | None
    time_constant_s: float | None
    settling_time_s: float | None
    time_to_half_s: float | None
    time_to_double_s: float | None
    dominant_state: str
    participation: tuple[tuple[str, float], ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "identifier": self.identifier,
            "name": self.name,
            "eigenvalues": [_complex_dict(value) for value in self.eigenvalues],
            "stability": self.stability,
            "response_type": self.response_type,
            "damping_character": self.damping_character,
            "natural_frequency_rad_s": self.natural_frequency_rad_s,
            "damped_frequency_rad_s": self.damped_frequency_rad_s,
            "damping_ratio": self.damping_ratio,
            "period_s": self.period_s,
            "time_constant_s": self.time_constant_s,
            "settling_time_s": self.settling_time_s,
            "time_to_half_s": self.time_to_half_s,
            "time_to_double_s": self.time_to_double_s,
            "dominant_state": self.dominant_state,
            "participation": dict(self.participation),
        }


@dataclass(frozen=True)
class ModalAnalysis:
    """Complete modal analysis for a continuous-time state matrix."""

    system_name: str
    eigenvalues: tuple[complex, ...]
    characteristic_polynomial: tuple[float, ...]
    modes: tuple[ModeResult, ...]
    spectral_abscissa: float
    stability: str
    eigenvector_condition_number: float

    def to_dict(self) -> dict[str, object]:
        return {
            "system_name": self.system_name,
            "eigenvalues": [_complex_dict(value) for value in self.eigenvalues],
            "characteristic_polynomial": list(self.characteristic_polynomial),
            "spectral_abscissa": self.spectral_abscissa,
            "stability": self.stability,
            "eigenvector_condition_number": self.eigenvector_condition_number,
            "modes": [mode.to_dict() for mode in self.modes],
        }


def _stability(real_parts: Iterable[float]) -> str:
    values = tuple(real_parts)
    if any(value > _ROOT_TOLERANCE for value in values):
        return "unstable"
    if all(value < -_ROOT_TOLERANCE for value in values):
        return "stable"
    return "marginal"


def modal_metrics_from_roots(
    identifier: str,
    name: str,
    roots: Iterable[complex],
    participation: dict[str, float],
) -> ModeResult:
    """Calculate damping and time metrics for one root or second-order pair."""

    eigenvalues = tuple(complex(value) for value in roots)
    if not eigenvalues:
        raise ValueError("a mode must contain at least one eigenvalue")
    if not participation:
        raise ValueError("mode participation cannot be empty")

    total_participation = sum(participation.values())
    if total_participation <= 0.0:
        raise ValueError("mode participation must have a positive sum")
    normalized_participation = tuple(
        (state, float(value / total_participation))
        for state, value in participation.items()
    )
    dominant_state = max(normalized_participation, key=lambda item: item[1])[0]
    stability = _stability(value.real for value in eigenvalues)
    governing_sigma = max(value.real for value in eigenvalues)

    complex_roots = [value for value in eigenvalues if abs(value.imag) > _ROOT_TOLERANCE]
    if complex_roots:
        representative = max(complex_roots, key=lambda value: value.imag)
        sigma = float(representative.real)
        damped_frequency = abs(float(representative.imag))
        natural_frequency = float(abs(representative))
        damping_ratio = -sigma / natural_frequency if natural_frequency else None
        response_type = "oscillatory"
        period = 2.0 * pi / damped_frequency
        if stability == "unstable":
            damping_character = "negative damping"
        elif stability == "marginal":
            damping_character = "undamped"
        elif damping_ratio is not None and damping_ratio < 1.0:
            damping_character = "underdamped"
        elif damping_ratio is not None and np.isclose(damping_ratio, 1.0, atol=1e-3):
            damping_character = "critically damped"
        else:
            damping_character = "overdamped"
    elif len(eigenvalues) == 2:
        roots_real = [float(value.real) for value in eigenvalues]
        product = roots_real[0] * roots_real[1]
        natural_frequency = sqrt(product) if product > 0.0 else None
        damping_ratio = (
            -(roots_real[0] + roots_real[1]) / (2.0 * natural_frequency)
            if natural_frequency
            else None
        )
        damped_frequency = 0.0
        period = None
        response_type = "aperiodic"
        if stability == "unstable":
            damping_character = "aperiodic divergence"
        elif damping_ratio is not None and np.isclose(damping_ratio, 1.0, atol=1e-3):
            damping_character = "critically damped"
        elif damping_ratio is not None and damping_ratio > 1.0:
            damping_character = "overdamped"
        else:
            damping_character = "aperiodic decay"
    else:
        natural_frequency = None
        damping_ratio = None
        damped_frequency = 0.0
        period = None
        response_type = "aperiodic"
        damping_character = {
            "stable": "aperiodic decay",
            "unstable": "aperiodic divergence",
            "marginal": "neutral",
        }[stability]

    time_constant = (
        1.0 / abs(governing_sigma)
        if abs(governing_sigma) > _ROOT_TOLERANCE
        else None
    )
    time_to_half = (
        log(2.0) / -governing_sigma
        if governing_sigma < -_ROOT_TOLERANCE
        else None
    )
    time_to_double = (
        log(2.0) / governing_sigma
        if governing_sigma > _ROOT_TOLERANCE
        else None
    )
    settling_time = (
        4.0 / -governing_sigma
        if governing_sigma < -_ROOT_TOLERANCE
        else None
    )

    return ModeResult(
        identifier=identifier,
        name=name,
        eigenvalues=eigenvalues,
        stability=stability,
        response_type=response_type,
        damping_character=damping_character,
        natural_frequency_rad_s=natural_frequency,
        damped_frequency_rad_s=damped_frequency,
        damping_ratio=damping_ratio,
        period_s=period,
        time_constant_s=time_constant,
        settling_time_s=settling_time,
        time_to_half_s=time_to_half,
        time_to_double_s=time_to_double,
        dominant_state=dominant_state,
        participation=normalized_participation,
    )


def _eigensystem(
    model: StateSpaceModel,
) -> tuple[ComplexArray, ComplexArray, FloatArray, float]:
    eigenvalues, right_vectors = np.linalg.eig(model.A)
    condition_number = float(np.linalg.cond(right_vectors))
    if not np.isfinite(condition_number):
        raise ValueError("eigenvector matrix is singular; modal participation is undefined")
    left_vectors = np.linalg.inv(right_vectors)
    participation = np.abs(right_vectors * left_vectors.T)
    column_sums = participation.sum(axis=0)
    participation = np.divide(
        participation,
        column_sums,
        out=np.zeros_like(participation, dtype=float),
        where=column_sums > 0.0,
    )
    return eigenvalues, right_vectors, participation, condition_number


def _root_groups(eigenvalues: ComplexArray) -> tuple[list[tuple[int, ...]], list[int]]:
    complex_groups: list[tuple[int, ...]] = []
    real_indices: list[int] = []
    used: set[int] = set()
    for index, value in enumerate(eigenvalues):
        if index in used:
            continue
        if abs(value.imag) <= _ROOT_TOLERANCE:
            real_indices.append(index)
            used.add(index)
            continue
        candidates = [
            other
            for other in range(index + 1, len(eigenvalues))
            if other not in used
        ]
        if not candidates:
            complex_groups.append((index,))
            used.add(index)
            continue
        partner = min(
            candidates,
            key=lambda other: abs(eigenvalues[other] - value.conjugate()),
        )
        if not np.isclose(
            eigenvalues[partner],
            value.conjugate(),
            rtol=1e-6,
            atol=1e-8,
        ):
            complex_groups.append((index,))
            used.add(index)
            continue
        complex_groups.append((index, partner))
        used.update((index, partner))
    return complex_groups, real_indices


def _group_participation(
    model: StateSpaceModel,
    participation_matrix: FloatArray,
    indices: tuple[int, ...],
) -> dict[str, float]:
    values = participation_matrix[:, indices].mean(axis=1)
    return {
        state: float(value)
        for state, value in zip(model.state_names, values, strict=True)
    }


def _make_mode(
    model: StateSpaceModel,
    eigenvalues: ComplexArray,
    participation_matrix: FloatArray,
    indices: tuple[int, ...],
    identifier: str,
    name: str,
) -> ModeResult:
    return modal_metrics_from_roots(
        identifier,
        name,
        (eigenvalues[index] for index in indices),
        _group_participation(model, participation_matrix, indices),
    )


def _longitudinal_modes(
    model: StateSpaceModel,
    eigenvalues: ComplexArray,
    participation_matrix: FloatArray,
) -> tuple[ModeResult, ...]:
    complex_groups, real_indices = _root_groups(eigenvalues)
    modes: list[ModeResult] = []

    if len(complex_groups) >= 2:
        ordered = sorted(
            complex_groups,
            key=lambda group: np.mean([abs(eigenvalues[index]) for index in group]),
        )
        modes.append(
            _make_mode(
                model,
                eigenvalues,
                participation_matrix,
                ordered[0],
                "phugoid",
                "Phugoid",
            )
        )
        modes.append(
            _make_mode(
                model,
                eigenvalues,
                participation_matrix,
                ordered[-1],
                "short_period",
                "Short period",
            )
        )
        for number, group in enumerate(ordered[1:-1], start=1):
            modes.append(
                _make_mode(
                    model,
                    eigenvalues,
                    participation_matrix,
                    group,
                    f"longitudinal_oscillatory_{number}",
                    f"Longitudinal oscillatory mode {number}",
                )
            )
    elif len(complex_groups) == 1:
        group = complex_groups[0]
        participation = _group_participation(model, participation_matrix, group)
        short_score = participation.get("w", 0.0) + participation.get("q", 0.0)
        phugoid_score = participation.get("u", 0.0) + participation.get("theta", 0.0)
        complex_is_short = short_score >= phugoid_score
        modes.append(
            _make_mode(
                model,
                eigenvalues,
                participation_matrix,
                group,
                "short_period" if complex_is_short else "phugoid",
                "Short period" if complex_is_short else "Phugoid",
            )
        )
        if real_indices:
            modes.append(
                _make_mode(
                    model,
                    eigenvalues,
                    participation_matrix,
                    tuple(real_indices),
                    "phugoid" if complex_is_short else "short_period",
                    "Phugoid (aperiodic)" if complex_is_short else "Short period (aperiodic)",
                )
            )
    elif len(real_indices) >= 4:
        scores = {}
        for index in real_indices:
            participation = _group_participation(
                model,
                participation_matrix,
                (index,),
            )
            scores[index] = participation.get("w", 0.0) + participation.get("q", 0.0)
        short_indices = tuple(sorted(real_indices, key=scores.get, reverse=True)[:2])
        phugoid_indices = tuple(index for index in real_indices if index not in short_indices)
        modes.extend(
            (
                _make_mode(
                    model,
                    eigenvalues,
                    participation_matrix,
                    short_indices,
                    "short_period",
                    "Short period (aperiodic)",
                ),
                _make_mode(
                    model,
                    eigenvalues,
                    participation_matrix,
                    phugoid_indices,
                    "phugoid",
                    "Phugoid (aperiodic)",
                ),
            )
        )
    else:
        for number, index in enumerate(real_indices, start=1):
            modes.append(
                _make_mode(
                    model,
                    eigenvalues,
                    participation_matrix,
                    (index,),
                    f"longitudinal_real_{number}",
                    f"Longitudinal real mode {number}",
                )
            )
    return tuple(modes)


def _lateral_modes(
    model: StateSpaceModel,
    eigenvalues: ComplexArray,
    participation_matrix: FloatArray,
) -> tuple[ModeResult, ...]:
    complex_groups, real_indices = _root_groups(eigenvalues)
    modes: list[ModeResult] = []

    if complex_groups:
        ordered_complex = sorted(
            complex_groups,
            key=lambda group: np.mean([abs(eigenvalues[index]) for index in group]),
        )
        for number, group in enumerate(ordered_complex, start=1):
            identifier = "dutch_roll" if number == 1 else f"lateral_oscillatory_{number}"
            name = "Dutch roll" if number == 1 else f"Lateral oscillatory mode {number}"
            modes.append(
                _make_mode(
                    model,
                    eigenvalues,
                    participation_matrix,
                    group,
                    identifier,
                    name,
                )
            )

    remaining = list(real_indices)
    if remaining:
        roll_index = max(
            remaining,
            key=lambda index: (
                abs(eigenvalues[index].real),
                participation_matrix[model.state_names.index("p"), index]
                if "p" in model.state_names
                else 0.0,
            ),
        )
        modes.append(
            _make_mode(
                model,
                eigenvalues,
                participation_matrix,
                (roll_index,),
                "roll_subsidence",
                "Roll subsidence",
            )
        )
        remaining.remove(roll_index)
    if remaining:
        spiral_index = min(
            remaining,
            key=lambda index: abs(eigenvalues[index].real),
        )
        modes.append(
            _make_mode(
                model,
                eigenvalues,
                participation_matrix,
                (spiral_index,),
                "spiral",
                "Spiral",
            )
        )
        remaining.remove(spiral_index)
    if remaining:
        modes.append(
            _make_mode(
                model,
                eigenvalues,
                participation_matrix,
                tuple(remaining),
                "dutch_roll_aperiodic",
                "Dutch roll (aperiodic)",
            )
        )
    return tuple(modes)


def _build_analysis(
    model: StateSpaceModel,
    modes: tuple[ModeResult, ...],
    eigenvalues: ComplexArray,
    condition_number: float,
) -> ModalAnalysis:
    polynomial = np.real_if_close(np.poly(eigenvalues), tol=1000)
    if np.iscomplexobj(polynomial):
        raise ValueError("real state matrix produced a non-real characteristic polynomial")
    spectral_abscissa = float(np.max(eigenvalues.real))
    return ModalAnalysis(
        system_name=model.name,
        eigenvalues=tuple(complex(value) for value in eigenvalues),
        characteristic_polynomial=tuple(float(value) for value in polynomial),
        modes=modes,
        spectral_abscissa=spectral_abscissa,
        stability=_stability(value.real for value in eigenvalues),
        eigenvector_condition_number=condition_number,
    )


def analyze_longitudinal_modes(model: StateSpaceModel) -> ModalAnalysis:
    """Analyze and classify the four-state longitudinal model."""

    eigenvalues, _, participation, condition_number = _eigensystem(model)
    modes = _longitudinal_modes(model, eigenvalues, participation)
    return _build_analysis(model, modes, eigenvalues, condition_number)


def analyze_lateral_modes(model: StateSpaceModel) -> ModalAnalysis:
    """Analyze and classify the four-state lateral-directional model."""

    eigenvalues, _, participation, condition_number = _eigensystem(model)
    modes = _lateral_modes(model, eigenvalues, participation)
    return _build_analysis(model, modes, eigenvalues, condition_number)


def format_modal_analysis(analysis: ModalAnalysis) -> str:
    """Return a concise, deterministic human-readable modal report."""

    lines = [
        f"{analysis.system_name}: {analysis.stability}",
        f"Spectral abscissa: {analysis.spectral_abscissa:.6f} 1/s",
    ]
    for mode in analysis.modes:
        roots = ", ".join(
            f"{value.real:+.6f}{value.imag:+.6f}j" for value in mode.eigenvalues
        )
        lines.extend(
            (
                f"  {mode.name}: {mode.stability}, {mode.damping_character}",
                f"    roots: {roots}",
                f"    dominant state: {mode.dominant_state}",
            )
        )
        if mode.natural_frequency_rad_s is not None:
            lines.append(f"    natural frequency: {mode.natural_frequency_rad_s:.6f} rad/s")
        if mode.damping_ratio is not None:
            lines.append(f"    damping ratio: {mode.damping_ratio:.6f}")
        if mode.period_s is not None:
            lines.append(f"    period: {mode.period_s:.6f} s")
        if mode.time_to_half_s is not None:
            lines.append(f"    time to half amplitude: {mode.time_to_half_s:.6f} s")
        if mode.time_to_double_s is not None:
            lines.append(f"    time to double amplitude: {mode.time_to_double_s:.6f} s")
    return "\n".join(lines)
