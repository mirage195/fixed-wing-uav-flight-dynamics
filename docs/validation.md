# Validation status

## Automated checks

The standard-library test suite currently verifies:

- lift, axial-force, and pitch-moment equilibrium at trim;
- normalized trim residual tolerance;
- rejection of aerodynamic lookup and speed-envelope extrapolation;
- rejection of in-envelope conditions that cannot satisfy trim constraints;
- rejection of non-finite aerodynamic derivatives;
- $Z_{\dot w}$, $M_{\dot w}$, and $V_0+Z_q$ matrix assembly;
- complete longitudinal and lateral $A,B,C,D$ dimensions;
- analytical $I_{xz}$ cross-inertia conversion;
- lateral control-matrix placement; and
- rejection of a non-positive-definite roll/yaw inertia block.

Phase 2 checks additionally verify:

- longitudinal phugoid and short-period identification;
- lateral Dutch-roll, roll-subsidence, and spiral identification;
- eigenvector participation normalization and dominant-state labels;
- stable, unstable, critically damped, and overdamped classifications;
- half-amplitude and doubling-time calculations;
- named initial-condition and control-step construction;
- time-response dimensions, finite results, and immutable result arrays; and
- JSON and CSV artifact schemas from the complete command-line workflow.

Phase 3 checks additionally verify:

- exact reproduction of the bundled case from the example TOML file;
- rejection of unknown configuration fields and unsupported schema versions;
- command-line configuration loading and headless artifact generation;
- rejection of contradictory command-line options; and
- agreement between the package version, entry point, and GPL release metadata.

Run:

```bash
python3 -m unittest discover -s tests -v
```

After installing the development dependencies, the release workflow uses:

```bash
ruff check .
pytest --cov=fixed_wing_uav --cov-report=term-missing --cov-fail-under=80
python -m build
twine check dist/*
```

## Illustrative-case baseline

With the bundled example, the bounded trim converges at:

| Quantity | Value |
|---|---:|
| Airspeed | 25.000 m/s |
| Angle of attack | 5.166 deg |
| Elevator | -1.649 deg |
| Throttle | 0.2632 |
| Thrust | 7.895 N |
| Maximum normalized equilibrium residual | $1.35\times10^{-16}$ |

This is a software regression baseline, not validation of a physical UAV.

The corresponding illustrative modal baseline is:

| Mode | Poles [1/s] | Damping ratio | Status | Key time metric |
|---|---:|---:|---|---:|
| Phugoid | $-0.012025 \pm 0.484091i$ | 0.0248 | Stable | 12.979 s period |
| Short period | $-3.019158 \pm 4.151110i$ | 0.5882 | Stable | 1.514 s period |
| Dutch roll | $-0.716593 \pm 6.691196i$ | 0.1065 | Stable | 0.939 s period |
| Roll subsidence | $-20.689046$ | N/A | Stable | 0.034 s half-time |
| Spiral | $+0.077964$ | N/A | **Unstable** | 8.891 s doubling time |

The positive spiral pole makes the complete lateral model unstable. This is an
intentional, visible result of the bundled illustrative coefficients; the
software does not suppress or relabel it.

## Evidence still required

Aircraft-specific validation requires:

1. Traceable geometry, mass, center-of-gravity, and inertia measurements.
2. Coefficient definitions with axes, signs, reference dimensions, and
   non-dimensional rate conventions.
3. Aerodynamic coefficient data over the complete requested trim envelope,
   including uncertainty.
4. A published or independently generated benchmark with expected dimensional
   derivatives and poles.
5. Comparison against nonlinear simulation, wind-tunnel data, or flight-test
   system identification.
6. Sensitivity and uncertainty analysis before handling-quality or controller
   conclusions are reported.

Until those items exist, the repository should describe numerical outputs as
illustrative rather than experimentally validated.
