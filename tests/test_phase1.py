"""Phase 1 physics, validation, and matrix-assembly tests."""

from __future__ import annotations

import unittest
from dataclasses import replace

import numpy as np

from fixed_wing_uav.aerodynamic_data import get_example_aerodynamics
from fixed_wing_uav.aircraft_parameters import Inertia, get_aircraft
from fixed_wing_uav.lateral_derivatives import compute_lateral_derivatives
from fixed_wing_uav.lateral_model import build_lateral_model
from fixed_wing_uav.longitudinal_model import build_longitudinal_model
from fixed_wing_uav.stability_derivatives import compute_all_derivatives
from fixed_wing_uav.trim import solve_level_trim


class PhaseOneTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.aircraft = get_aircraft()
        self.aerodynamics = get_example_aerodynamics()
        self.trim = solve_level_trim(self.aircraft, self.aerodynamics)
        self.longitudinal_derivatives = compute_all_derivatives(
            self.aircraft,
            self.trim,
            self.aerodynamics,
        )
        self.lateral_derivatives = compute_lateral_derivatives(
            self.aircraft,
            self.aerodynamics,
            self.trim,
        )

    def test_trim_satisfies_force_and_moment_equilibrium(self) -> None:
        qbar = self.trim.dynamic_pressure
        weight = self.aircraft.mass * self.aircraft.gravity
        self.assertAlmostEqual(qbar * self.aircraft.S * self.trim.CL0, weight, places=10)
        self.assertAlmostEqual(
            self.trim.thrust0,
            qbar * self.aircraft.S * self.trim.CD0,
            places=10,
        )
        self.assertAlmostEqual(self.trim.Cm0, 0.0, places=12)
        self.assertLess(self.trim.maximum_normalized_residual, 1e-10)

    def test_aerodynamic_lookup_refuses_extrapolation(self) -> None:
        with self.assertRaises(ValueError):
            self.aerodynamics.coefficients(self.aerodynamics.alpha[-1] + 0.01)

    def test_out_of_envelope_speed_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            solve_level_trim(self.aircraft, self.aerodynamics, speed=15.0)

    def test_in_envelope_but_untrimmable_condition_is_rejected(self) -> None:
        with self.assertRaises(RuntimeError):
            solve_level_trim(self.aircraft, self.aerodynamics, speed=20.0)

    def test_nonfinite_derivative_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            replace(self.aerodynamics.longitudinal, CL_q=np.nan)

    def test_longitudinal_matrix_uses_implicit_derivatives(self) -> None:
        derivatives = self.longitudinal_derivatives
        model = build_longitudinal_model(self.aircraft, self.trim, derivatives)
        denominator = 1.0 - derivatives.Zw_dot
        expected_zq_entry = (self.trim.V0 + derivatives.Zq) / denominator

        self.assertEqual(model.A.shape, (4, 4))
        self.assertEqual(model.B.shape, (4, 2))
        self.assertEqual(model.C.shape, (4, 4))
        self.assertEqual(model.D.shape, (4, 2))
        self.assertAlmostEqual(model.A[1, 2], expected_zq_entry)
        self.assertAlmostEqual(model.A[0, 3], -self.aircraft.gravity)
        self.assertAlmostEqual(model.A[1, 3], 0.0)
        self.assertAlmostEqual(
            model.A[2, 0],
            derivatives.Mu + derivatives.Mw_dot * model.A[1, 0],
        )
        self.assertAlmostEqual(
            model.B[2, 0],
            derivatives.M_delta_e + derivatives.Mw_dot * model.B[1, 0],
        )

    def test_lateral_derivatives_apply_cross_inertia(self) -> None:
        lat = self.aerodynamics.lateral
        qbar = self.trim.dynamic_pressure
        moment_scale = qbar * self.aircraft.S * self.aircraft.b / self.trim.V0
        raw_roll = moment_scale * lat.Cl_beta
        raw_yaw = moment_scale * lat.Cn_beta
        inertia = self.aircraft.inertia
        gamma = inertia.determinant_xz
        expected_Lv = (inertia.Izz * raw_roll + inertia.Ixz * raw_yaw) / gamma
        expected_Nv = (inertia.Ixz * raw_roll + inertia.Ixx * raw_yaw) / gamma

        self.assertAlmostEqual(self.lateral_derivatives.Lv, expected_Lv)
        self.assertAlmostEqual(self.lateral_derivatives.Nv, expected_Nv)
        self.assertNotAlmostEqual(self.lateral_derivatives.Lv, raw_roll / inertia.Ixx)

    def test_lateral_model_contains_control_matrix(self) -> None:
        model = build_lateral_model(
            self.aircraft,
            self.trim,
            self.lateral_derivatives,
        )
        self.assertEqual(model.A.shape, (4, 4))
        self.assertEqual(model.B.shape, (4, 2))
        self.assertAlmostEqual(model.A[0, 2], self.lateral_derivatives.Yr - self.trim.V0)
        self.assertAlmostEqual(model.A[3, 2], 0.0)
        self.assertAlmostEqual(model.B[1, 0], self.lateral_derivatives.L_delta_a)
        self.assertTrue(np.allclose(model.C, np.eye(4)))

    def test_invalid_inertia_tensor_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Inertia(Ixx=1.0, Iyy=1.0, Izz=1.0, Ixz=1.1)


if __name__ == "__main__":
    unittest.main()
