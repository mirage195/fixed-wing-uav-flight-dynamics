from src.aircraft_parameters import get_aircraft
from src.stability_derivatives import compute_all_derivatives, TrimCondition
from src.longitudinal_model import build_longitudinal_model
from src.lateral_derivatives import compute_lateral_derivatives
from src.lateral_model import build_lateral_model
from src.eigen_analysis import (
    analyze_longitudinal_modes,
    analyze_lateral_modes,
    plot_eigenvalues
)

import numpy as np


def main():

    # =========================
    # AIRCRAFT
    # =========================
    aircraft = get_aircraft()

    # =========================
    # TRIM
    # =========================
    trim = TrimCondition(
        V0=50.0,
        alpha0=np.deg2rad(2),
        CL0=0.4,
        CD0=0.03,
        rho=1.225,
        S=aircraft.S,
        c_bar=aircraft.c
    )

    # =========================
    # AERO DATA
    # =========================
    aero_data = {
        "V_samples": np.array([40,45,50,55,60]),
        "CL_vs_V": np.array([0.42,0.40,0.39,0.385,0.38]),
        "CD_vs_V": np.array([0.035,0.034,0.033,0.0325,0.032]),
        "alpha_samples": np.deg2rad(np.array([-2,0,2,4,6])),
        "CL_vs_alpha": np.array([0.1,0.25,0.40,0.55,0.70]),
        "CD_vs_alpha": np.array([0.03,0.031,0.033,0.037,0.044]),
        "Cm_vs_V": np.array([-0.02,-0.0205,-0.021,-0.0212,-0.0214]),
        "Cm_vs_alpha": np.array([0.04,0.02,0.00,-0.02,-0.04]),
        "d_eps_dalpha": 0.4
    }

    # =========================
    # LONGITUDINAL DERIVATIVES
    # =========================
    long_der = compute_all_derivatives(
        trim=trim,
        tail=aircraft.tail,
        aero_data=aero_data
    )

    print("\n=== LONGITUDINAL DERIVATIVES ===")
    for k, v in long_der.items():
        print(f"{k}: {v:.4f}")

    # =========================
    # LATERAL DERIVATIVES
    # =========================
    lat_der = compute_lateral_derivatives(
        aircraft,
        aero_data={
            "h": np.linspace(0,1,10),
            "c_h": np.ones(10)*0.5,
            "a_h": np.ones(10)*3.5,
            "a_y": np.ones_like(aircraft.wing.y_span)*5.7,
            "CD_y": np.ones_like(aircraft.wing.y_span)*0.08,
            "CL_y": np.ones_like(aircraft.wing.y_span)*0.6,
            "dCD_dalpha_y": np.ones_like(aircraft.wing.y_span)*0.1
        },
        CL_trim=0.4
    )

    print("\n=== LATERAL DERIVATIVES ===")
    for k, v in lat_der.items():
        print(f"{k}: {v:.4f}")

    # =========================
    # BUILD MODELS
    # =========================
    A_lon = build_longitudinal_model(aircraft, long_der)
    A_lat = build_lateral_model(aircraft, lat_der)

    # =========================
    # EIGEN ANALYSIS
    # =========================

    # -------- LONGITUDINAL --------
    print("\n=== LONGITUDINAL MODES ===")
    analyze_longitudinal_modes(A_lon)
    plot_eigenvalues(A_lon)

    # -------- LATERAL --------
    print("\n=== LATERAL MODES ===")
    analyze_lateral_modes(A_lat)
    plot_eigenvalues(A_lat)


if __name__ == "__main__":
    main()