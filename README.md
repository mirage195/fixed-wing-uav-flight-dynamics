# Fixed-Wing UAV Flight Dynamics

[![CI](https://github.com/mirage195/fixed-wing-uav-flight-dynamics/actions/workflows/ci.yml/badge.svg)](https://github.com/mirage195/fixed-wing-uav-flight-dynamics/actions/workflows/ci.yml)
[![License: GPL v3+](https://img.shields.io/badge/License-GPL_v3%2B-blue.svg)](LICENSE)
[![Python: 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)](pyproject.toml)

A Python project for building and inspecting linear longitudinal and
lateral-directional models of a fixed-wing aircraft about a converged trim
condition.

> **Project status:** Phase 3 installable analysis package. The bundled aircraft and
> aerodynamic values are illustrative and are not flight validated. Do not use
> the generated models, responses, or future controller gains in a flight
> vehicle without an independent engineering verification program.

## Implemented

- Validated aircraft, inertia, propulsion, control-limit, and aerodynamic data
  models
- Bounded straight-flight trim for angle of attack, elevator, and throttle
- Explicit conversion from non-dimensional coefficients to SI acceleration
  derivatives
- Complete longitudinal and lateral $A,B,C,D$ state-space matrices
- Downwash-rate terms and $I_{xz}$ roll/yaw cross-inertia coupling
- Structured open-loop modal analysis with damping, frequency, period, and
  time-domain metrics
- Eigenvector participation factors and conventional mode identification
- Initial-condition and control-step simulations
- Deterministic JSON, CSV, PNG, and SVG analysis artifacts
- Strict, versioned TOML configuration with explicit units
- Installable `fixed_wing_uav` package and `uav-dynamics` command
- Automated tests, linting, distribution builds, and GitHub issue templates
- Open-source contribution, security, citation, and release documentation
- GNU General Public License v3.0 or later

The exact coordinate, sign, rate-normalization, and dimensional-derivative
definitions are in [docs/conventions.md](docs/conventions.md). The larger
project scope is tracked in [docs/design-scope.md](docs/design-scope.md). Modal
metrics and classification rules are documented in
[docs/analysis.md](docs/analysis.md).

## Installation

Python 3.10 or newer is required. Clone the repository and install it in an
isolated environment:

```bash
git clone https://github.com/mirage195/fixed-wing-uav-flight-dynamics.git
cd fixed-wing-uav-flight-dynamics
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
```

## Command-line usage

Run the bundled illustrative case:

```bash
uav-dynamics
```

Run an explicit TOML configuration, choose an output directory, or omit plots:

```bash
uav-dynamics --config configs/illustrative_aircraft.toml
uav-dynamics --output-dir my-results
uav-dynamics --no-plots
```

For a headless run that still creates figures:

```bash
MPLBACKEND=Agg uav-dynamics
```

The legacy `python run_full_analysis.py` launcher remains available in a
repository checkout after installation. Run `uav-dynamics --help` for every
option. The configuration schema and unit rules are documented in
[docs/configuration.md](docs/configuration.md).

## Current model

The straight-flight trim solver prescribes airspeed and flight-path angle and
solves:

$$
L-W\cos\gamma_0=0,\qquad
T-D-W\sin\gamma_0=0,\qquad
M=0.
$$

The resulting small-disturbance models are

$$
x_{lon}=[u,w,q,\theta]^T,\quad
u_{lon}=[\delta_e,\delta_T]^T,
$$

$$
x_{lat}=[v,p,r,\phi]^T,\quad
u_{lat}=[\delta_a,\delta_r]^T.
$$

Each builder returns a validated continuous-time state-space object containing
$A,B,C,D$, state names, input names, and output names.

## Generated artifacts

The default run produces:

```text
results/
├── analysis_summary.json
├── figures/
│   ├── longitudinal_modes.{png,svg}
│   ├── lateral_modes.{png,svg}
│   ├── longitudinal_{initial,elevator_step}.{png,svg}
│   └── lateral_{initial,rudder_step}.{png,svg}
└── tables/
    ├── modal_summary.csv
    └── *.csv
```

The JSON report contains the trim point, derivative sets, complete state-space
matrices, modal results, and simulation summaries. Simulation CSV files retain
the full input, state, and output histories for independent plotting or review.

## Python API

```python
from fixed_wing_uav import default_configuration, run_analysis

results = run_analysis(
    default_configuration(),
    output_dir="results",
    generate_plots=False,
)
print(results.lateral_analysis.stability)
```

Public data objects are validated dataclasses, and state-space and simulation
arrays are immutable after construction.

## Example-data warning

The values returned by `get_aircraft()` and
`get_example_aerodynamics()` preserve the prototype as an executable
example. Missing control derivatives were added only to exercise the complete
matrix interface. Replace the example with traceable wind-tunnel, CFD, system
identification, or flight-test data before interpreting the results as an
aircraft prediction.

## Roadmap

1. Handling-quality assessment against a precisely cited applicable standard
2. LQR/LQI/PID synthesis after controllability and model validation
3. Nonlinear 6-DoF dynamics and maneuver trim

## Development and licensing

Install the development tools and run the local checks with:

```bash
python -m pip install -e ".[dev]"
ruff check .
pytest --cov=fixed_wing_uav --cov-report=term-missing --cov-fail-under=80
python -m build
twine check dist/*
```

Contributions are welcome; see [CONTRIBUTING.md](CONTRIBUTING.md). This project
is copyright 2026 Harshit Beri and distributed under the
[GNU General Public License v3.0 or later](LICENSE). Distributed derivative
works must follow the license's source and copyleft requirements.

See [docs/validation.md](docs/validation.md) for current verification coverage
and remaining evidence requirements.
