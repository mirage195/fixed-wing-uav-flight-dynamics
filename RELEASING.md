# Release process

1. Confirm the test, lint, and package jobs pass on the intended release
   commit.
2. Move the relevant changelog entries into a dated version section.
3. Update the version in `pyproject.toml`, `CITATION.cff`, and
   `fixed_wing_uav.__version__`.
4. Build and inspect the distributions:

   ```bash
   python -m build
   twine check dist/*
   ```

5. Commit the release metadata and create a signed `vX.Y.Z` tag.
6. Push the tag and create a GitHub release from the matching changelog entry.
7. Publish to PyPI only after the project name and trusted-publishing settings
   are registered. Never place a PyPI token in the repository.

Releases follow Semantic Versioning. Before version 1.0, minor releases may
contain documented public-API changes.
