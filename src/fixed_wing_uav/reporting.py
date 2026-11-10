"""Deterministic reports and figures for modal and time-response analysis."""

from __future__ import annotations

import csv
import json
from collections.abc import Iterable, Mapping
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .aerodynamic_data import AerodynamicData
from .aircraft_parameters import Aircraft
from .eigen_analysis import ModalAnalysis
from .simulation import SimulationResult
from .state_space import StateSpaceModel
from .trim import TrimCondition

_STATE_UNITS = {
    "u": "m/s",
    "v": "m/s",
    "w": "m/s",
    "p": "rad/s",
    "q": "rad/s",
    "r": "rad/s",
    "phi": "rad",
    "theta": "rad",
    "psi": "rad",
}


def _json_ready(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    return value


def _model_dict(model: StateSpaceModel) -> dict[str, object]:
    return {
        "name": model.name,
        "state_names": list(model.state_names),
        "input_names": list(model.input_names),
        "output_names": list(model.output_names),
        "A": model.A.tolist(),
        "B": model.B.tolist(),
        "C": model.C.tolist(),
        "D": model.D.tolist(),
    }


def _simulation_summary(result: SimulationResult) -> dict[str, object]:
    states = {}
    for index, name in enumerate(result.state_names):
        history = result.states[:, index]
        states[name] = {
            "initial": float(history[0]),
            "final": float(history[-1]),
            "minimum": float(np.min(history)),
            "maximum": float(np.max(history)),
            "peak_absolute": float(np.max(np.abs(history))),
            "unit": _STATE_UNITS.get(name, "unknown"),
        }
    return {
        "name": result.name,
        "duration_s": float(result.time[-1] - result.time[0]),
        "sample_count": int(result.time.size),
        "states": states,
    }


def write_json_report(
    path: str | Path,
    *,
    aircraft: Aircraft,
    aerodynamics: AerodynamicData,
    trim: TrimCondition,
    derivatives: Mapping[str, object],
    models: Mapping[str, StateSpaceModel],
    analyses: Mapping[str, ModalAnalysis],
    simulations: Mapping[str, SimulationResult],
    data_status: str = "illustrative; not flight validated",
    configuration_schema_version: int = 1,
) -> Path:
    """Write the complete numerical Phase 2 summary without timestamps."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "data_status": data_status,
        "configuration_schema_version": configuration_schema_version,
        "aircraft": {
            "name": aircraft.name,
            "provenance": aircraft.provenance,
            "mass_kg": aircraft.mass,
            "reference_speed_m_s": aircraft.reference_speed,
            "air_density_kg_m3": aircraft.air_density,
        },
        "aerodynamics": {
            "provenance": aerodynamics.provenance,
            "speed_range_m_s": list(aerodynamics.speed_range),
            "alpha_range_rad": [
                float(aerodynamics.alpha[0]),
                float(aerodynamics.alpha[-1]),
            ],
        },
        "trim": asdict(trim),
        "derivatives": {
            name: values.as_dict() for name, values in derivatives.items()
        },
        "models": {name: _model_dict(model) for name, model in models.items()},
        "modal_analysis": {
            name: analysis.to_dict() for name, analysis in analyses.items()
        },
        "simulations": {
            name: _simulation_summary(result)
            for name, result in simulations.items()
        },
    }
    destination.write_text(
        json.dumps(_json_ready(payload), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return destination


def write_modal_csv(
    path: str | Path,
    analyses: Iterable[ModalAnalysis],
) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = (
        "system",
        "mode",
        "identifier",
        "stability",
        "response_type",
        "damping_character",
        "eigenvalues",
        "natural_frequency_rad_s",
        "damped_frequency_rad_s",
        "damping_ratio",
        "period_s",
        "time_constant_s",
        "settling_time_s",
        "time_to_half_s",
        "time_to_double_s",
        "dominant_state",
        "participation",
    )
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for analysis in analyses:
            for mode in analysis.modes:
                writer.writerow(
                    {
                        "system": analysis.system_name,
                        "mode": mode.name,
                        "identifier": mode.identifier,
                        "stability": mode.stability,
                        "response_type": mode.response_type,
                        "damping_character": mode.damping_character,
                        "eigenvalues": ";".join(
                            f"{root.real:+.12g}{root.imag:+.12g}j"
                            for root in mode.eigenvalues
                        ),
                        "natural_frequency_rad_s": mode.natural_frequency_rad_s,
                        "damped_frequency_rad_s": mode.damped_frequency_rad_s,
                        "damping_ratio": mode.damping_ratio,
                        "period_s": mode.period_s,
                        "time_constant_s": mode.time_constant_s,
                        "settling_time_s": mode.settling_time_s,
                        "time_to_half_s": mode.time_to_half_s,
                        "time_to_double_s": mode.time_to_double_s,
                        "dominant_state": mode.dominant_state,
                        "participation": json.dumps(
                            dict(mode.participation),
                            sort_keys=True,
                        ),
                    }
                )
    return destination


def write_simulation_csv(path: str | Path, result: SimulationResult) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    headers = ["time_s"]
    headers.extend(f"input:{name}" for name in result.input_names)
    headers.extend(f"state:{name}" for name in result.state_names)
    headers.extend(f"output:{name}" for name in result.output_names)
    combined = np.column_stack(
        (result.time, result.inputs, result.states, result.outputs)
    )
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        writer.writerows(combined)
    return destination


def plot_eigenvalues(analysis: ModalAnalysis):
    import matplotlib.pyplot as plt

    with plt.rc_context({"figure.figsize": (7.0, 5.0), "svg.hashsalt": "uav-dynamics"}):
        figure, axes = plt.subplots()
        roots = np.asarray(analysis.eigenvalues)
        axes.scatter(roots.real, roots.imag, color="#0068b5", s=45, zorder=3)
        for mode in analysis.modes:
            for root in mode.eigenvalues:
                axes.annotate(
                    mode.name,
                    (root.real, root.imag),
                    xytext=(5, 5),
                    textcoords="offset points",
                    fontsize=8,
                )
        axes.axvline(0.0, color="black", linewidth=1.0)
        axes.axhline(0.0, color="black", linewidth=0.6)
        axes.set_xlabel("Real part [1/s]")
        axes.set_ylabel("Imaginary part [rad/s]")
        axes.set_title(f"{analysis.system_name} modes ({analysis.stability})")
        axes.grid(True, alpha=0.35)
        figure.tight_layout()
    return figure


def plot_simulation(result: SimulationResult):
    import matplotlib.pyplot as plt

    row_count = len(result.state_names)
    with plt.rc_context(
        {
            "figure.figsize": (8.0, 2.25 * row_count),
            "svg.hashsalt": "uav-dynamics",
        }
    ):
        figure, axes = plt.subplots(row_count, 1, sharex=True, squeeze=False)
        for index, state_name in enumerate(result.state_names):
            axis = axes[index, 0]
            axis.plot(result.time, result.states[:, index], color="#0068b5")
            unit = _STATE_UNITS.get(state_name, "")
            axis.set_ylabel(f"{state_name} [{unit}]" if unit else state_name)
            axis.grid(True, alpha=0.35)
        axes[0, 0].set_title(result.name)
        axes[-1, 0].set_xlabel("Time [s]")
        figure.tight_layout()
    return figure


def save_figure_formats(
    figure,
    output_stem: str | Path,
    *,
    close: bool = True,
) -> tuple[Path, Path]:
    """Save deterministic PNG and SVG variants and close the figure."""

    import matplotlib.pyplot as plt

    stem = Path(output_stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    png_path = stem.with_suffix(".png")
    svg_path = stem.with_suffix(".svg")
    figure.savefig(
        png_path,
        dpi=160,
        bbox_inches="tight",
        metadata={"Software": "fixed-wing-uav-flight-dynamics"},
    )
    with plt.rc_context({"svg.hashsalt": "uav-dynamics"}):
        figure.savefig(
            svg_path,
            bbox_inches="tight",
            metadata={"Date": None},
        )
    if close:
        plt.close(figure)
    return png_path, svg_path
