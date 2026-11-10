"""Fixed-wing UAV trim, linearization, and dynamics analysis."""

__version__ = "0.1.0"

from .aerodynamic_data import AerodynamicData, get_example_aerodynamics
from .aircraft_parameters import Aircraft, get_aircraft
from .configuration import (
    AnalysisConfiguration,
    ConfigurationError,
    default_configuration,
    load_configuration,
)
from .eigen_analysis import (
    ModalAnalysis,
    ModeResult,
    analyze_lateral_modes,
    analyze_longitudinal_modes,
    format_modal_analysis,
    modal_metrics_from_roots,
)
from .lateral_derivatives import LateralDerivatives, compute_lateral_derivatives
from .lateral_model import build_lateral_model
from .longitudinal_model import build_longitudinal_model
from .simulation import (
    SimulationResult,
    simulate_initial_response,
    simulate_response,
    simulate_step_response,
    state_vector,
    step_inputs,
    time_vector,
)
from .stability_derivatives import LongitudinalDerivatives, compute_all_derivatives
from .state_space import StateSpaceModel
from .trim import TrimCondition, solve_level_trim
from .workflow import AnalysisResults, build_analysis, run_analysis

__all__ = [
    "__version__",
    "AerodynamicData",
    "Aircraft",
    "AnalysisConfiguration",
    "AnalysisResults",
    "ConfigurationError",
    "LateralDerivatives",
    "LongitudinalDerivatives",
    "ModalAnalysis",
    "ModeResult",
    "SimulationResult",
    "StateSpaceModel",
    "TrimCondition",
    "analyze_lateral_modes",
    "analyze_longitudinal_modes",
    "build_lateral_model",
    "build_analysis",
    "build_longitudinal_model",
    "compute_all_derivatives",
    "compute_lateral_derivatives",
    "default_configuration",
    "format_modal_analysis",
    "get_aircraft",
    "get_example_aerodynamics",
    "modal_metrics_from_roots",
    "load_configuration",
    "run_analysis",
    "solve_level_trim",
    "simulate_initial_response",
    "simulate_response",
    "simulate_step_response",
    "state_vector",
    "step_inputs",
    "time_vector",
]
