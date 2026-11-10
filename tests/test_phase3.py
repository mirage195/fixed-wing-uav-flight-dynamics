"""Phase 3 configuration, CLI, and release-metadata tests."""

from __future__ import annotations

import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import asdict
from io import StringIO
from pathlib import Path

import numpy as np

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - exercised on Python 3.10
    import tomli as tomllib

from fixed_wing_uav import __version__
from fixed_wing_uav.cli import main
from fixed_wing_uav.configuration import (
    CONFIG_SCHEMA_VERSION,
    ConfigurationError,
    configuration_from_mapping,
    default_configuration,
    load_configuration,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
EXAMPLE_CONFIG = PROJECT_ROOT / "configs" / "illustrative_aircraft.toml"


class PhaseThreeTestCase(unittest.TestCase):
    def test_example_toml_reproduces_bundled_configuration(self) -> None:
        loaded = load_configuration(EXAMPLE_CONFIG)
        expected = default_configuration()

        self.assertEqual(loaded.schema_version, CONFIG_SCHEMA_VERSION)
        self.assertEqual(loaded.data_status, expected.data_status)
        self.assertEqual(loaded.aircraft, expected.aircraft)
        self.assertEqual(loaded.aerodynamics.provenance, expected.aerodynamics.provenance)
        self.assertEqual(loaded.aerodynamics.speed_range, expected.aerodynamics.speed_range)
        self.assertTrue(np.array_equal(loaded.aerodynamics.alpha, expected.aerodynamics.alpha))
        self.assertTrue(np.array_equal(loaded.aerodynamics.CL, expected.aerodynamics.CL))
        self.assertTrue(np.array_equal(loaded.aerodynamics.CD, expected.aerodynamics.CD))
        self.assertTrue(np.array_equal(loaded.aerodynamics.Cm, expected.aerodynamics.Cm))
        self.assertEqual(
            asdict(loaded.aerodynamics.longitudinal),
            asdict(expected.aerodynamics.longitudinal),
        )
        self.assertEqual(
            asdict(loaded.aerodynamics.lateral),
            asdict(expected.aerodynamics.lateral),
        )

    def test_unknown_configuration_key_is_rejected(self) -> None:
        parsed = tomllib.loads(EXAMPLE_CONFIG.read_text(encoding="utf-8"))
        parsed["aircraft"]["mispeled_mass"] = 13.5

        with self.assertRaisesRegex(ConfigurationError, "unknown keys"):
            configuration_from_mapping(parsed)

    def test_unsupported_configuration_schema_is_rejected(self) -> None:
        parsed = tomllib.loads(EXAMPLE_CONFIG.read_text(encoding="utf-8"))
        parsed["schema_version"] = CONFIG_SCHEMA_VERSION + 1

        with self.assertRaisesRegex(ConfigurationError, "unsupported schema_version"):
            configuration_from_mapping(parsed)

    def test_cli_accepts_configuration_and_writes_reports(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = StringIO()
            with redirect_stdout(output):
                status = main(
                    [
                        "--config",
                        str(EXAMPLE_CONFIG),
                        "--output-dir",
                        temporary_directory,
                        "--no-plots",
                    ]
                )

            report = json.loads(
                (Path(temporary_directory) / "analysis_summary.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(status, 0)
            self.assertIn("Illustrative 13.5 kg fixed-wing UAV", output.getvalue())
            self.assertEqual(report["configuration_schema_version"], 1)
            self.assertEqual(report["data_status"], "illustrative; not flight validated")
            self.assertFalse((Path(temporary_directory) / "figures").exists())

    def test_conflicting_plot_options_are_rejected(self) -> None:
        with redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit) as raised:
                main(["--no-plots", "--show-plots"])
        self.assertEqual(raised.exception.code, 2)

    def test_release_metadata_is_consistent_and_gpl(self) -> None:
        metadata = tomllib.loads(
            (PROJECT_ROOT / "pyproject.toml").read_text(encoding="utf-8")
        )["project"]
        license_text = (PROJECT_ROOT / "LICENSE").read_text(encoding="utf-8")

        self.assertEqual(metadata["version"], __version__)
        self.assertEqual(metadata["license"], "GPL-3.0-or-later")
        self.assertEqual(
            metadata["scripts"]["uav-dynamics"],
            "fixed_wing_uav.cli:main",
        )
        self.assertIn("GNU GENERAL PUBLIC LICENSE", license_text)
        self.assertIn("Version 3, 29 June 2007", license_text)


if __name__ == "__main__":
    unittest.main()
