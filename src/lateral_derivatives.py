import numpy as np

# ============================================================
# HELPER
# ============================================================

def _integral(y, f):
    return float(np.trapz(f, y))


# ============================================================
# Yv (Sideforce due to sideslip)
# ============================================================

def compute_Yv(aircraft):
    S = aircraft.S
    fus = aircraft.fuselage
    fin = aircraft.fin

    Yv = (fus.SB / S) * fus.yB - (fin.SF / S) * fin.a1F
    return Yv


# ============================================================
# Lv (Rolling moment due to sideslip)
# ============================================================

def compute_Lv(aircraft, CL_trim):

    wing = aircraft.wing
    dihedral = aircraft.dihedral
    fin = aircraft.fin

    S = aircraft.S
    b = aircraft.b
    s = b / 2

    y = wing.y_span
    c_y = wing.c_y
    a_y = wing.a_y
    gamma = dihedral.gamma_y

    # ---- Dihedral contribution ----
    integrand = c_y * a_y * gamma * y
    Lv_dihedral = -_integral(y, integrand) / (S * s)

    # ---- Sweep contribution ----
    sweep = np.deg2rad(wing.sweep_25_deg)
    integrand = c_y * y
    Lv_sweep = -(2 * CL_trim * np.tan(sweep) / (S * s)) * _integral(y, integrand)

    # ---- Fin contribution ----
    Lv_fin = -(fin.SF * fin.hF / (S * b)) * fin.a1F

    return Lv_dihedral + Lv_sweep + Lv_fin


# ============================================================
# Nv (Yawing moment due to sideslip)
# ============================================================

def compute_Nv(aircraft):
    S = aircraft.S
    b = aircraft.b
    fin = aircraft.fin

    VF = (fin.SF * fin.lF) / (S * b)
    Nv = VF * fin.a1F

    return Nv


# ============================================================
# Yp (Sideforce due to roll rate)
# ============================================================

def compute_Yp(aircraft, h, c_h, a_h):

    S = aircraft.S
    b = aircraft.b

    integrand = a_h * c_h * h
    Yp = -_integral(h, integrand) / (S * b)

    return Yp


# ============================================================
# Lp (Roll damping)
# ============================================================

def compute_Lp(aircraft, y, c_y, a_y, CD_y):

    S = aircraft.S
    s = aircraft.b / 2

    integrand = (a_y + CD_y) * c_y * (y**2)
    Lp = -_integral(y, integrand) / (2 * S * s**2)

    return Lp


# ============================================================
# Np (Yawing moment due to roll rate)
# ============================================================

def compute_Np(aircraft, y, c_y, CL_y, dCD_dalpha_y):

    S = aircraft.S
    s = aircraft.b / 2

    integrand = (CL_y - dCD_dalpha_y) * c_y * (y**2)
    Np = -_integral(y, integrand) / (2 * S * s**2)

    return Np


# ============================================================
# Yr (Sideforce due to yaw rate)
# ============================================================

def compute_Yr(aircraft):

    S = aircraft.S
    b = aircraft.b
    fin = aircraft.fin

    VF = (fin.SF * fin.lF) / (S * b)
    Yr = VF * fin.a1F

    return Yr


# ============================================================
# Lr (Rolling moment due to yaw rate)
# ============================================================

def compute_Lr(aircraft, y, c_y, CL_y):

    S = aircraft.S
    s = aircraft.b / 2

    integrand = CL_y * c_y * (y**2)
    Lr_wing = _integral(y, integrand) / (S * s**2)

    # Fin contribution
    fin = aircraft.fin
    VF = (fin.SF * fin.lF) / (S * aircraft.b)
    Lr_fin = fin.a1F * VF * (fin.hF / aircraft.b)

    return Lr_wing + Lr_fin


# ============================================================
# Nr (Yaw damping)
# ============================================================

def compute_Nr(aircraft, y, c_y, CD_y):

    S = aircraft.S
    s = aircraft.b / 2

    integrand = CD_y * c_y * (y**2)
    Nr_wing = -_integral(y, integrand) / (S * s**2)

    # Fin contribution
    fin = aircraft.fin
    VF = (fin.SF * fin.lF) / (S * aircraft.b)
    Nv = VF * fin.a1F

    Nr_fin = -(fin.lF / aircraft.b) * Nv

    return Nr_wing + Nr_fin


# ============================================================
# MASTER FUNCTION
# ============================================================

def compute_lateral_derivatives(aircraft, aero_data, CL_trim):

    y = aircraft.wing.y_span
    c_y = aircraft.wing.c_y

    return {
        "Yv": compute_Yv(aircraft),
        "Lv": compute_Lv(aircraft, CL_trim),
        "Nv": compute_Nv(aircraft),

        "Yp": compute_Yp(aircraft, aero_data["h"], aero_data["c_h"], aero_data["a_h"]),
        "Lp": compute_Lp(aircraft, y, c_y, aero_data["a_y"], aero_data["CD_y"]),
        "Np": compute_Np(aircraft, y, c_y, aero_data["CL_y"], aero_data["dCD_dalpha_y"]),

        "Yr": compute_Yr(aircraft),
        "Lr": compute_Lr(aircraft, y, c_y, aero_data["CL_y"]),
        "Nr": compute_Nr(aircraft, y, c_y, aero_data["CD_y"])
    }
