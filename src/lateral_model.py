import numpy as np

def build_lateral_model(aircraft, der):
    """
    Build lateral-directional state matrix (A_lat)

    States:
    [v, p, r, phi]

    v   : lateral velocity
    p   : roll rate
    r   : yaw rate
    phi : roll angle
    """

    m = aircraft.mass
    Ix = aircraft.Ix
    Iz = aircraft.Iz
    S = aircraft.S
    b = aircraft.b
    rho = 1.225
    V0 = 50.0

    # Extract derivatives
    Yv = der["Yv"]
    Yp = der["Yp"]
    Yr = der["Yr"]

    Lv = der["Lv"]
    Lp = der["Lp"]
    Lr = der["Lr"]

    Nv = der["Nv"]
    Np = der["Np"]
    Nr = der["Nr"]

    # Build A matrix
    A = np.array([
        [Yv/m, Yp/m, Yr/m - V0, 9.81],
        [Lv/Ix, Lp/Ix, Lr/Ix, 0],
        [Nv/Iz, Np/Iz, Nr/Iz, 0],
        [0, 1, 0, 0]
    ])

    return A
