"""Phase 2 modal-analysis, simulation, and reporting tests."""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from math import log
from pathlib import Path

import numpy as np

from fixed_wing_uav.aerodynamic_data import get_example_aerodynamics
from fixed_wing_uav.aircraft_parameters import get_aircraft
from fixed_wing_uav.configuration import default_configuration
from fixed_wing_uav.eigen_analysis import (
    analyze_lateral_modes,
    analyze_longitudinal_modes,
    modal_metrics_from_roots,
)
from fixed_wing_uav.lateral_derivatives import compute_lateral_derivatives
from fixed_wing_uav.lateral_model import build_lateral_model
from fixed_wing_uav.longitudinal_model import build_longitudinal_model
from fixed_wing_uav.simulation import (
    simulate_initial_response,
    simulate_step_response,
    step_inputs,
    time_vector,
)
from fixed_wing_uav.stability_derivatives import compute_all_derivatives
from fixed_wing_uav.trim import solve_level_trim
from fixed_wing_uav.workflow import run_analysis


class PhaseTwoTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.aircraft = get_aircraft()
        cls.aerodynamics = get_example_aerodynamics()
        cls.trim = solve_level_trim(cls.aircraft, cls.aerodynamics)
        cls.longitudinal_derivatives = compute_all_derivatives(
            cls.aircraft,
            cls.trim,
            cls.aerodynamics,
        )
        cls.lateral_derivatives = compute_lateral_derivatives(
            cls.aircraft,
            cls.aerodynamics,
            cls.trim,
        )
        cls.longitudinal_model = build_longitudinal_model(
            cls.aircraft,
            cls.trim,
            cls.longitudinal_derivatives,
        )
        cls.lateral_model = build_lateral_model(
            cls.aircraft,
            cls.trim,
            cls.lateral_derivatives,
        )

    def test_longitudinal_modes_are_classified_and_stable(self) -> None:
        analysis = analyze_longitudinal_modes(self.longitudinal_model)

        self.assertEqual(analysis.stability, "stable")
        self.assertEqual(
            {mode.identifier for mode in analysis.modes},
            {"phugoid", "short_period"},
        )
        for mode in analysis.modes:
            self.assertEqual(mode.response_type, "oscillatory")
            self.assertEqual(mode.damping_character, "underdamped")
            self.assertGreater(mode.natural_frequency_rad_s, 0.0)
            self.assertGreater(mode.period_s, 0.0)
            self.assertGreater(mode.time_to_half_s, 0.0)
            self.assertIsNone(mode.time_to_double_s)
            self.assertAlmostEqual(sum(dict(mode.participation).values()), 1.0)

    def test_lateral_modes_detect_spiral_instability(self) -> None:
        analysis = analyze_lateral_modes(self.lateral_model)
        modes = {mode.identifier: mode for mode in analysis.modes}

        self.assertEqual(analysis.stability, "unstable")
        self.assertEqual(
            set(modes),
            {"dutch_roll", "roll_subsidence", "spiral"},
        )
        self.assertEqual(modes["dutch_roll"].dominant_state, "r")
        self.assertEqual(modes["roll_subsidence"].dominant_state, "p")
        self.assertEqual(modes["spiral"].stability, "unstable")
        self.assertIsNone(modes["spiral"].time_to_half_s)
        self.assertGreater(modes["spiral"].time_to_double_s, 0.0)

    def test_real_second_order_damping_categories(self) -> None:
        critical = modal_metrics_from_roots(
            "critical",
            "Critical",
            (-1.0, -1.0),
            {"x": 1.0},
        )
        overdamped = modal_metrics_from_roots(
            "overdamped",
            "Overdamped",
            (-1.0, -3.0),
            {"x": 1.0},
        )
        divergence = modal_metrics_from_roots(
            "divergence",
            "Divergence",
            (0.2,),
            {"x": 1.0},
        )

        self.assertEqual(critical.damping_character, "critically damped")
        self.assertAlmostEqual(critical.damping_ratio, 1.0)
        self.assertEqual(overdamped.damping_character, "overdamped")
        self.assertGreater(overdamped.damping_ratio, 1.0)
        self.assertEqual(divergence.stability, "unstable")
        self.assertAlmostEqual(divergence.time_to_double_s, log(2.0) / 0.2)

    def test_initial_condition_response_preserves_initial_state(self) -> None:
        time = time_vector(1.0, 0.02)
        result = simulate_initial_response(
            self.longitudinal_model,
            time,
            {"u": 1.0, "theta": 0.1},
        )

        self.assertEqual(result.states.shape, (time.size, 4))
        self.assertEqual(result.inputs.shape, (time.size, 2))
        self.assertTrue(np.allclose(result.states[0], (1.0, 0.0, 0.0, 0.1)))
        self.assertFalse(result.states.flags.writeable)
        self.assertTrue(np.all(np.isfinite(result.states)))

    def test_control_step_starts_at_requested_time(self) -> None:
        time = time_vector(1.0, 0.1)
        amplitude = np.deg2rad(-1.0)
        inputs = step_inputs(
            self.longitudinal_model,
            time,
            {"elevator": (amplitude, 0.5)},
        )
        result = simulate_step_response(
            self.longitudinal_model,
            time,
            {"elevator": (amplitude, 0.5)},
        )

        elevator = self.longitudinal_model.input_names.index("elevator")
        throttle = self.longitudinal_model.input_names.index("throttle")
        self.assertTrue(np.all(inputs[time < 0.5, elevator] == 0.0))
        self.assertTrue(np.all(inputs[time >= 0.5, elevator] == amplitude))
        self.assertTrue(np.all(inputs[:, throttle] == 0.0))
        self.assertTrue(np.allclose(result.inputs, inputs))

    def test_full_analysis_writes_machine_readable_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            destination = Path(temporary_directory)
            with redirect_stdout(StringIO()):
                run_analysis(
                    default_configuration(),
                    output_dir=destination,
                    generate_plots=False,
                )

            report_path = destination / "analysis_summary.json"
            modal_path = destination / "tables" / "modal_summary.csv"
            payload = json.loads(report_path.read_text(encoding="utf-8"))
            with modal_path.open(encoding="utf-8", newline="") as stream:
                modal_rows = list(csv.DictReader(stream))

            self.assertEqual(payload["schema_version"], 1)
            self.assertEqual(
                set(payload["modal_analysis"]),
                {"longitudinal", "lateral"},
            )
            self.assertEqual(len(modal_rows), 5)
            self.assertEqual(
                {row["identifier"] for row in modal_rows},
                {
                    "phugoid",
                    "short_period",
                    "dutch_roll",
                    "roll_subsidence",
                    "spiral",
                },
            )
            for name in payload["simulations"]:
                self.assertTrue((destination / "tables" / f"{name}.csv").is_file())


if __name__ == "__main__":
    unittest.main()
