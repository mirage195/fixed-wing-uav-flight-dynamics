import numpy as np
from dataclasses import dataclass

# ============================================================
# WING
# ============================================================

@dataclass
class MainWing:
    S: float
    b: float
    s: float
    sweep_25_deg: float
    y_span: np.ndarray
    c_y: np.ndarray
    a_y: np.ndarray


@dataclass
class DihedralDistribution:
    gamma_y: np.ndarray


# ============================================================
# FUSELAGE
# ============================================================

@dataclass
class Fuselage:
    SB: float
    yB: float


# ============================================================
# FIN
# ============================================================

@dataclass
class Fin:
    SF: float
    lF: float
    hF: float
    a1F: float


# ============================================================
# TAIL
# ============================================================

@dataclass
class AircraftTailData:
    ST: float
    lT: float
    c_bar: float
    a1: float
    dCDT_dalphaT: float


# ============================================================
# FULL AIRCRAFT
# ============================================================

@dataclass
class Aircraft:
    mass: float
    Ix: float
    Iy: float
    Iz: float
    Ixz: float
    S: float
    b: float
    c: float
    U0: float
    rho: float
    wing: MainWing
    dihedral: DihedralDistribution
    fuselage: Fuselage
    fin: Fin
    tail: AircraftTailData


def get_aircraft():

    mass = 13.5
    Ix = 0.8244
    Iy = 1.135
    Iz = 1.759
    Ixz = 0.1204

    S = 0.55
    b = 2.90
    c = 0.19

    U0 = 18.0
    rho = 1.225

    y_span = np.linspace(0, b/2, 20)
    c_y = np.full_like(y_span, c)
    a_y = np.full_like(y_span, 5.7)

    wing = MainWing(S, b, b/2, 0.0, y_span, c_y, a_y)

    dihedral = DihedralDistribution(np.zeros_like(y_span))

    fuselage = Fuselage(0.2, -0.02)

    fin = Fin(0.1, 0.8, 0.1, 2.5)

    tail = AircraftTailData(0.12, 0.9, c, 3.5, 0.0)

    aircraft = Aircraft(
        mass, Ix, Iy, Iz, Ixz,
        S, b, c, U0, rho,
        wing, dihedral, fuselage, fin, tail
    )

    return aircraft