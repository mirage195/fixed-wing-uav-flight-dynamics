from src.aircraft_parameters import get_aircraft
from src.stability_derivatives import compute_all_derivatives, TrimCondition
from src.longitudinal_model import build_longitudinal_model
from src.lateral_derivatives import compute_lateral_derivatives
from src.eigen_analysis import analyze_longitudinal_modes, plot_eigenvalues

import numpy as np


def main():

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
    # AERO DATA (replace later)
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
    # LONGITUDINAL
    # =========================
    derivatives = compute_all_derivatives(
        trim=trim,
        tail=aircraft.tail,
        aero_data=aero_data
    )

    A_lon = build_longitudinal_model(aircraft, derivatives)

    print("\n=== LONGITUDINAL MODES ===")
    analyze_longitudinal_modes(A_lon)
    plot_eigenvalues(A_lon)

    # =========================
    # LATERAL
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
    print(lat_der)


if __name__ == "__main__":
    main()
