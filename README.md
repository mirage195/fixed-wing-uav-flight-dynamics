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

## Results and Analysis

### Longitudinal Derivatives

```
Xu: -0.0500
Zu: -0.7000
Xw: 0.3427
Zw: -4.3272
Mu: -0.0050
Mw: -0.5730
Xq: -0.0000
Zq: -3.6172
Mq: -17.1342
Xw_dot: -0.0000
Zw_dot: -1.4469
Mw_dot: -6.8537
```

---

### Lateral-Directional Derivatives

```
Yv: -0.4618
Lv: -0.0157
Nv: 0.1254
Yp: -0.5486
Lp: -0.4832
Np: -0.0418
Yr: 0.1254
Lr: 0.1046
Nr: -0.0480
```

---

### Longitudinal Modes (Eigenvalue Analysis)

```
Eigenvalues:
[-15.1051, -0.4131, 0.0489 ± 0.1912j]
```

**Interpretation:**
- Fast real mode corresponds to **short-period damping**
- Slow real mode represents **longitudinal decay**
- Complex pair represents **phugoid mode**

**Key Observation:**
- The phugoid mode has a **positive real part**, indicating **instability**
- This suggests insufficient aerodynamic damping at the chosen trim condition

---

### Lateral-Directional Modes

```
Eigenvalues:
[-0.0061 ± 1.9022j, -0.6699, 0.0345]
```

**Mode Identification:**
- **Dutch Roll Mode:**  
  Complex conjugate pair with:
  - Natural frequency ≈ 1.90 rad/s  
  - Very low damping ratio ≈ 0.003  
  → Indicates **lightly damped oscillatory behavior**

- **Roll Mode:**  
  Real negative eigenvalue ≈ -0.67  
  → Fast and stable roll damping

- **Spiral Mode:**  
  Real positive eigenvalue ≈ +0.0345  
  → Indicates **spiral instability**

---

### Key Engineering Insights

- The aircraft exhibits:
  - **Unstable phugoid mode**
  - **Unstable spiral mode**
  - **Lightly damped Dutch roll**

- Stability characteristics are highly sensitive to:
  - Drag modeling
  - Tail volume and effectiveness
  - Trim conditions

- These results highlight the need for:
  - Improved aerodynamic design  
  - or feedback control (e.g., LQR / PID)

---

### Visualization

Eigenvalue plots are generated for both longitudinal and lateral systems using Matplotlib.


