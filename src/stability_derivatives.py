import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

# ============================================================
# TRIM CONDITION
# ============================================================

@dataclass
class TrimCondition:
    V0: float
    alpha0: float
    CL0: float
    CD0: float
    rho: float
    S: float
    c_bar: float

    @property
    def Sc(self):
        return self.S * self.c_bar


# ============================================================
# HELPER FUNCTION
# ============================================================

def _finite_derivative(x: np.ndarray, y: np.ndarray, x0: float) -> float:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    idx = np.argsort(x)
    x_sorted = x[idx]
    y_sorted = y[idx]

    if x0 <= x_sorted[0]:
        i0, i1 = 0, 1
    elif x0 >= x_sorted[-1]:
        i0, i1 = -2, -1
    else:
        i1 = np.searchsorted(x_sorted, x0)
        i0 = i1 - 1

    x1, x2 = x_sorted[i0], x_sorted[i1]
    y1, y2 = y_sorted[i0], y_sorted[i1]

    return (y2 - y1) / (x2 - x1)


# ============================================================
# FORCE DERIVATIVES
# ============================================================

def compute_force_derivatives(
    trim: TrimCondition,
    V_samples, CL_vs_V, CD_vs_V,
    alpha_samples, CL_vs_alpha, CD_vs_alpha,
    thrust_vs_V=None
):

    V0 = trim.V0
    CL0 = trim.CL0
    CD0 = trim.CD0
    rho = trim.rho
    S = trim.S

    q0 = 0.5 * rho * V0**2

    dCD_dV = _finite_derivative(V_samples, CD_vs_V, V0)
    dCL_dV = _finite_derivative(V_samples, CL_vs_V, V0)

    dCD_dalpha = _finite_derivative(alpha_samples, CD_vs_alpha, trim.alpha0)
    dCL_dalpha = _finite_derivative(alpha_samples, CL_vs_alpha, trim.alpha0)

    if thrust_vs_V is not None:
        dTau_dV = _finite_derivative(V_samples, thrust_vs_V, V0)
    else:
        dTau_dV = 0.0

    Xu = -2*CD0 - V0*dCD_dV + dTau_dV/(q0*S)
    Zu = -2*CL0 - V0*dCL_dV
    Xw = CL0 - dCD_dalpha
    Zw = -(dCL_dalpha + CD0)

    return Xu, Zu, Xw, Zw


# ============================================================
# MOMENT DERIVATIVES
# ============================================================

def compute_moment_derivatives(
    trim: TrimCondition,
    V_samples, Cm_vs_V,
    alpha_samples, Cm_vs_alpha
):

    dCm_dV = _finite_derivative(V_samples, Cm_vs_V, trim.V0)
    dCm_dalpha = _finite_derivative(alpha_samples, Cm_vs_alpha, trim.alpha0)

    Mu = trim.V0 * dCm_dV
    Mw = dCm_dalpha

    return Mu, Mw


# ============================================================
# PITCH RATE DERIVATIVES
# ============================================================

def compute_pitch_rate_derivatives(trim, tail):

    S = trim.S
    c = trim.c_bar

    ST = tail.ST
    lT = tail.lT
    a1 = tail.a1
    dCDT = tail.dCDT_dalphaT

    V_T = (ST * lT) / (S * c)

    Xq = -V_T * dCDT
    Zq = -V_T * a1
    Mq = -V_T * (lT / c) * a1

    return Xq, Zq, Mq


# ============================================================
# W-DOT DERIVATIVES
# ============================================================

def compute_wdot_derivatives(trim, tail, d_eps_dalpha):

    Xq, Zq, Mq = compute_pitch_rate_derivatives(trim, tail)

    Xw_dot = Xq * d_eps_dalpha
    Zw_dot = Zq * d_eps_dalpha
    Mw_dot = Mq * d_eps_dalpha

    return Xw_dot, Zw_dot, Mw_dot


# ============================================================
# MASTER FUNCTION
# ============================================================

def compute_all_derivatives(
    trim,
    tail,
    aero_data
):

    Xu, Zu, Xw, Zw = compute_force_derivatives(
        trim,
        aero_data["V_samples"],
        aero_data["CL_vs_V"],
        aero_data["CD_vs_V"],
        aero_data["alpha_samples"],
        aero_data["CL_vs_alpha"],
        aero_data["CD_vs_alpha"],
        aero_data.get("thrust_vs_V", None)
    )

    Mu, Mw = compute_moment_derivatives(
        trim,
        aero_data["V_samples"],
        aero_data["Cm_vs_V"],
        aero_data["alpha_samples"],
        aero_data["Cm_vs_alpha"]
    )

    Xq, Zq, Mq = compute_pitch_rate_derivatives(trim, tail)

    Xw_dot, Zw_dot, Mw_dot = compute_wdot_derivatives(
        trim, tail, aero_data["d_eps_dalpha"]
    )

    return {
        "Xu": Xu, "Zu": Zu, "Xw": Xw, "Zw": Zw,
        "Mu": Mu, "Mw": Mw,
        "Xq": Xq, "Zq": Zq, "Mq": Mq,
        "Xw_dot": Xw_dot,
        "Zw_dot": Zw_dot,
        "Mw_dot": Mw_dot
    }
