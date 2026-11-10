"""End-to-end construction, simulation, and artifact generation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .configuration import AnalysisConfiguration
from .eigen_analysis import (
    ModalAnalysis,
    analyze_lateral_modes,
    analyze_longitudinal_modes,
    format_modal_analysis,
)
from .lateral_derivatives import LateralDerivatives, compute_lateral_derivatives
from .lateral_model import build_lateral_model
from .longitudinal_model import build_longitudinal_model
from .reporting import (
    plot_eigenvalues,
    plot_simulation,
    save_figure_formats,
    write_json_report,
    write_modal_csv,
    write_simulation_csv,
)
from .simulation import (
    SimulationResult,
    simulate_initial_response,
    simulate_step_response,
    time_vector,
)
from .stability_derivatives import LongitudinalDerivatives, compute_all_derivatives
from .state_space import StateSpaceModel
from .trim import TrimCondition, solve_level_trim


@dataclass(frozen=True)
class AnalysisResults:
    """Numerical results produced before report serialization."""

    configuration: AnalysisConfiguration
    trim: TrimCondition
    longitudinal_derivatives: LongitudinalDerivatives
    lateral_derivatives: LateralDerivatives
    longitudinal_model: StateSpaceModel
    lateral_model: StateSpaceModel
    longitudinal_analysis: ModalAnalysis
    lateral_analysis: ModalAnalysis
    simulations: dict[str, SimulationResult]


def build_analysis(configuration: AnalysisConfiguration) -> AnalysisResults:
    """Build all Phase 1 and Phase 2 results without writing files."""

    aircraft = configuration.aircraft
    aerodynamics = configuration.aerodynamics
    trim = solve_level_trim(aircraft, aerodynamics)
    longitudinal_derivatives = compute_all_derivatives(
        aircraft=aircraft,
        trim=trim,
        aero_data=aerodynamics,
    )
    lateral_derivatives = compute_lateral_derivatives(
        aircraft=aircraft,
        aero_data=aerodynamics,
        trim=trim,
    )
    longitudinal_model = build_longitudinal_model(
        aircraft,
        trim,
        longitudinal_derivatives,
    )
    lateral_model = build_lateral_model(
        aircraft,
        trim,
        lateral_derivatives,
    )
    longitudinal_analysis = analyze_longitudinal_modes(longitudinal_model)
    lateral_analysis = analyze_lateral_modes(lateral_model)
    longitudinal_time = time_vector(30.0, 0.02)
    lateral_time = time_vector(15.0, 0.01)
    simulations = {
        "longitudinal_initial": simulate_initial_response(
            longitudinal_model,
            longitudinal_time,
            {"u": 1.0, "theta": np.deg2rad(2.0)},
            name="Longitudinal initial-condition response",
        ),
        "longitudinal_elevator_step": simulate_step_response(
            longitudinal_model,
            longitudinal_time,
            {"elevator": (np.deg2rad(-1.0), 1.0)},
            name="Longitudinal response to -1 deg elevator step",
        ),
        "lateral_initial": simulate_initial_response(
            lateral_model,
            lateral_time,
            {"v": 0.5, "phi": np.deg2rad(5.0)},
            name="Lateral initial-condition response",
        ),
        "lateral_rudder_step": simulate_step_response(
            lateral_model,
            lateral_time,
            {"rudder": (np.deg2rad(1.0), 1.0)},
            name="Lateral response to +1 deg rudder step",
        ),
    }
    return AnalysisResults(
        configuration=configuration,
        trim=trim,
        longitudinal_derivatives=longitudinal_derivatives,
        lateral_derivatives=lateral_derivatives,
        longitudinal_model=longitudinal_model,
        lateral_model=lateral_model,
        longitudinal_analysis=longitudinal_analysis,
        lateral_analysis=lateral_analysis,
        simulations=simulations,
    )


def write_analysis_artifacts(
    results: AnalysisResults,
    output_dir: str | Path,
    *,
    generate_plots: bool = True,
    show_plots: bool = False,
) -> Path:
    """Write deterministic machine-readable reports and optional figures."""

    destination = Path(output_dir)
    figures_directory = destination / "figures"
    tables_directory = destination / "tables"
    destination.mkdir(parents=True, exist_ok=True)
    analyses = {
        "longitudinal": results.longitudinal_analysis,
        "lateral": results.lateral_analysis,
    }
    models = {
        "longitudinal": results.longitudinal_model,
        "lateral": results.lateral_model,
    }
    derivatives = {
        "longitudinal": results.longitudinal_derivatives,
        "lateral": results.lateral_derivatives,
    }
    write_json_report(
        destination / "analysis_summary.json",
        aircraft=results.configuration.aircraft,
        aerodynamics=results.configuration.aerodynamics,
        trim=results.trim,
        derivatives=derivatives,
        models=models,
        analyses=analyses,
        simulations=results.simulations,
        data_status=results.configuration.data_status,
        configuration_schema_version=results.configuration.schema_version,
    )
    write_modal_csv(tables_directory / "modal_summary.csv", analyses.values())
    for name, simulation in results.simulations.items():
        write_simulation_csv(tables_directory / f"{name}.csv", simulation)

    if generate_plots:
        figures = {
            "longitudinal_modes": plot_eigenvalues(results.longitudinal_analysis),
            "lateral_modes": plot_eigenvalues(results.lateral_analysis),
        }
        figures.update(
            {
                name: plot_simulation(simulation)
                for name, simulation in results.simulations.items()
            }
        )
        for name, figure in figures.items():
            save_figure_formats(
                figure,
                figures_directory / name,
                close=not show_plots,
            )
        if show_plots:
            import matplotlib.pyplot as plt

            plt.show()
            plt.close("all")
    return destination.resolve()


def run_analysis(
    configuration: AnalysisConfiguration,
    output_dir: str | Path = "results",
    *,
    generate_plots: bool = True,
    show_plots: bool = False,
) -> AnalysisResults:
    """Build an analysis, write its artifacts, and return numerical results."""

    results = build_analysis(configuration)
    write_analysis_artifacts(
        results,
        output_dir,
        generate_plots=generate_plots,
        show_plots=show_plots,
    )
    return results


def format_analysis_summary(results: AnalysisResults, output_dir: str | Path) -> str:
    """Return the concise terminal summary for one completed run."""

    trim = results.trim
    lines = [
        f"Aircraft: {results.configuration.aircraft.name}",
        f"Data status: {results.configuration.data_status}",
        "",
        "=== TRIM ===",
        f"Speed: {trim.V0:.3f} m/s",
        f"Angle of attack: {np.degrees(trim.alpha0):.3f} deg",
        f"Elevator: {np.degrees(trim.elevator0):.3f} deg",
        f"Throttle: {trim.throttle0:.4f}",
        f"Maximum normalized residual: {trim.maximum_normalized_residual:.3e}",
        "",
        "=== MODAL ANALYSIS ===",
        format_modal_analysis(results.longitudinal_analysis),
        "",
        format_modal_analysis(results.lateral_analysis),
        "",
        f"Reports written to: {Path(output_dir).resolve()}",
    ]
    return "\n".join(lines)
