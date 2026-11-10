# TOML configuration

The command-line interface accepts a versioned TOML file:

```bash
uav-dynamics --config configs/illustrative_aircraft.toml
```

The bundled file reproduces the built-in illustrative case and is intended as
a complete template. Copy it before entering aircraft-specific data.

## Conventions

- Dimensional values use SI units identified in each field name.
- Human-entered geometry and control angles use degrees where the name ends in
  `_deg`; they are converted to radians during loading.
- Aerodynamic angular derivatives are per radian.
- Rotary derivatives use $pb/(2V)$, $qc/(2V)$, and $rb/(2V)$.
- Control derivatives are per radian, except normalized throttle.
- `speed_range_m_s` and the angle-of-attack table define the accepted data
  envelope. The analysis refuses extrapolation.

The complete force, moment, axis, and sign definitions are in
[conventions.md](conventions.md).

## Validation behavior

`schema_version = 1` is mandatory. Every documented table and field must be
present, including derivative values that are zero. Unknown keys are rejected
to catch spelling mistakes instead of silently ignoring them. Existing
dataclass validation then checks positivity, finiteness, array sizes, inertia
properties, control bounds, and monotonic lookup axes.

The `data_status` string is copied into the JSON report and terminal summary.
Use a precise statement such as `illustrative; not flight validated` until the
complete aircraft data set has traceable evidence.

## Reproducibility

Commit source configurations when their data may be shared. Record proprietary
or restricted inputs outside the repository and retain their hashes and
provenance in the appropriate controlled system. Generated `results/` files
are ignored by Git because they can be recreated from a committed configuration
and software version.
