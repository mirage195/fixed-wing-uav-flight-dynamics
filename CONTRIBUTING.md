# Contributing

Contributions are welcome through GitHub issues and pull requests. By
submitting a contribution, you agree that it may be distributed under the
project's GNU General Public License v3.0 or later.

## Development setup

Fork and clone the repository, then create an isolated environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

Run the same core checks used in continuous integration:

```bash
ruff check .
pytest --cov=fixed_wing_uav --cov-report=term-missing --cov-fail-under=80
python -m build
twine check dist/*
```

## Engineering expectations

- State coordinate systems, signs, reference quantities, and units explicitly.
- Add a traceable source and validity envelope for physical input data.
- Preserve the distinction between software regression and aircraft
  validation.
- Add focused tests for equations, indexing, failure cases, and public APIs.
- Do not silently extrapolate aerodynamic data or suppress unstable modes.
- Keep generated artifacts out of commits unless they are deliberate reviewed
  fixtures.

Small, focused pull requests are easier to review. Open an issue before a
large architectural change or the addition of a handling-quality standard.
