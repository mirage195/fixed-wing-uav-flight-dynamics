# Fixed-Wing UAV Flight Dynamics & Stability Analysis

## Overview
This project presents a complete flight dynamics modeling and stability analysis framework for a fixed-wing UAV.

The model includes:
- Aerodynamic force and moment modeling
- Stability derivative computation
- Linearized state-space representation
- Eigenvalue-based flight mode analysis

---

## Key Features

### Aircraft Modeling
- Parametric geometry definition
- Wing, tail, fuselage, and fin contributions
- Spanwise aerodynamic distribution support

### Stability Derivatives
- Computed from aerodynamic data (CL, CD, Cm)
- Includes:
  - Longitudinal derivatives (Xu, Zw, Mq, etc.)
  - Lateral derivatives (Yv, Lv, Nr, etc.)
- Tail and downwash effects included

### Flight Dynamics Model
- Linearized 6-DOF system split into:
  - Longitudinal dynamics
  - Lateral-directional dynamics

### Eigenvalue Analysis
- Automatic mode identification:
  - Phugoid
  - Short-period
  - Roll mode
  - Spiral mode
  - Dutch roll

---

## Example Output

### Longitudinal Eigenvalues

```
[-15.10, -0.41, 0.048 ± 0.191j]
```

### Key Observation

The phugoid mode exhibits slight instability due to low aerodynamic damping, highlighting sensitivity to drag modeling and trim conditions.

