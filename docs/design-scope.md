# Design scope and implementation status

The working ODT supplied with the project describes a seven-stage concept:
parameter ingestion, trim, dimensional derivatives, state-space assembly,
modal analysis, handling-quality assessment, and controller synthesis.

It is treated as a design input, not as evidence that every stage is already
implemented or technically verified.

## Phase 1 implemented

- Validated aircraft, inertia, control-limit, propulsion, and aerodynamic data
  models
- Prescribed-speed straight-flight trim with bounds and residual checks
- Explicit non-dimensional coefficient conventions
- SI longitudinal and lateral acceleration derivatives
- Longitudinal $A,B,C,D$ matrices including $Z_{\dot w}$,
  $M_{\dot w}$, $V_0+Z_q$, and trim-angle gravity terms
- Lateral $A,B,C,D$ matrices including $I_{xz}$ roll/yaw coupling
- An executable illustrative case and automated Phase 1 tests

## Phase 2 implemented

- Structured eigenvalue and characteristic-polynomial results
- Natural and damped frequencies, damping ratios, periods, time constants,
  approximate settling times, and half/doubling times where applicable
- Stable, marginal, and unstable classifications with explicit aperiodic,
  underdamped, critically damped, and overdamped descriptions
- Right/left-eigenvector participation factors and conventional longitudinal
  and lateral mode labels
- Initial-condition and named control-step simulations
- Versioned JSON summaries, CSV tables, and PNG/SVG figures
- Automated modal, simulation, and reporting regression tests

## Phase 3 implemented

- Standard `src/fixed_wing_uav` package layout and `pyproject.toml` metadata
- `uav-dynamics` console entry point and `python -m fixed_wing_uav` execution
- Strict schema-versioned TOML input with human-readable units
- Reusable build/write workflow separated from the command-line interface
- Python 3.10, 3.11, and 3.12 GitHub Actions test matrix
- Lint, coverage, wheel/source-distribution, and metadata checks
- GPL-3.0-or-later licensing, citation metadata, changelog, and release process
- Contribution, conduct, security, issue, and pull-request guidance

## Deliberately not claimed yet

- General nonlinear 12-state or 6-DoF equations of motion
- Maneuver, asymmetric, or turning trim
- Validated geometry-to-derivative prediction
- Handling-quality compliance reports
- LQR, LQI, or PID synthesis
- Flight-controller parameter export

Those capabilities remain roadmap items. In particular, statements in the
working document about handling-quality compliance, universal LQR robustness
margins, or direct flight-computer deployment are not adopted by this
implementation without cited requirements and an appropriate verification
program.

Mode names in Phase 2 are engineering classifications based on pole location
and state participation, not proof of compliance with a handling-quality
standard. The exact heuristics and metrics are recorded in
[analysis.md](analysis.md).

## Data status

The bundled aircraft and aerodynamic coefficients are illustrative. Most
values were inherited from the prototype; missing control derivatives were
added solely to exercise the complete matrix interface. Results must not be
described as predictions for a physical aircraft until each value has a
traceable source, coordinate/sign convention, uncertainty, and valid flight
envelope.
