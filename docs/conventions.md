# Mathematical and coordinate conventions

This document is normative for the implementation. Every input data source must
be converted to these conventions before it is used.

## Reference axes and signs

The reduced linear models use a right-handed stability-axis frame at the trim
condition:

- $x_S$: aligned with the steady flight-velocity vector
- $y_S$: right
- $z_S$: down

The translational perturbations are $u,v,w$ along those axes. Angular rates
$p,q,r$ follow the right-hand rule. Positive $\phi$ is right-wing-down roll,
positive $\theta$ is nose-up pitch perturbation, and positive $\psi$ is
nose-right yaw.

The physical body reference line is separated from $x_S$ by the trim angle of
attack $\alpha_0$. The trim object therefore stores both flight-path angle
$\gamma_0$ and physical pitch attitude $\theta_0=\alpha_0+\gamma_0$. Gravity
and kinematic entries in the reduced stability-axis matrices use $\gamma_0$.
Input inertias and aerodynamic derivatives must be expressed in the same
analysis axes; a source using body or wind axes must be transformed first.

Aerodynamic forces $X,Y,Z$ and moments $L,M,N$ use the positive stability
axes. Lift $L_A$ is positive upward, so its dominant small-angle contribution
is $Z=-L_A$. Angles and angular derivatives are always in radians.

The example uses positive elevator deflection as trailing-edge down. Aileron
and rudder signs must be recorded with each coefficient source; the code does
not silently reverse supplied control derivatives.

## Coefficient normalization

Dynamic pressure is

$$
\bar q = \frac{1}{2}\rho V_0^2.
$$

Rotary coefficient derivatives use

$$
\hat p=\frac{pb}{2V_0},\qquad
\hat q=\frac{q\bar c}{2V_0},\qquad
\hat r=\frac{rb}{2V_0}.
$$

Angle-of-attack-rate derivatives use

$$
\widehat{\dot\alpha}=\frac{\dot\alpha\bar c}{2V_0}.
$$

For example, $C_{mq}=\partial C_m/\partial\hat q$, not a derivative with
respect to dimensional $q$.

## Dimensional-derivative convention

The derivative dataclasses contain **acceleration derivatives**. Force
derivatives have already been divided by mass. Pitch derivatives have already
been divided by $I_{yy}$, and lateral moment derivatives have already been
resolved through the coupled $I_{xx},I_{zz},I_{xz}$ inertia equations.
State-space builders therefore insert these values directly and never divide
them by mass or inertia a second time.

Representative conversions are

$$
Y_v=\frac{\bar qS}{mV_0}C_{Y\beta},\qquad
Y_p=\frac{\bar qS}{m}\frac{b}{2V_0}C_{Yp},
$$

$$
Z_q=-\frac{\bar qS}{m}\frac{\bar c}{2V_0}C_{Lq},\qquad
M_q=\frac{\bar qS\bar c}{I_{yy}}
\frac{\bar c}{2V_0}C_{mq}.
$$

For an unnormalized rolling/yawing moment-derivative pair
$(\mathcal L_i,\mathcal N_i)$, cross-inertia coupling is

$$
\Gamma=I_{xx}I_{zz}-I_{xz}^2,
$$

$$
L_i'=\frac{I_{zz}\mathcal L_i+I_{xz}\mathcal N_i}{\Gamma},
\qquad
N_i'=\frac{I_{xz}\mathcal L_i+I_{xx}\mathcal N_i}{\Gamma}.
$$

## Trim definition

Phase 1 supports prescribed-speed, straight flight. It solves for
$\alpha_0,\delta_{e0},\delta_{T0}$ subject to control and aerodynamic-data
bounds:

$$
L-W\cos\gamma_0=0,\qquad
T-D-W\sin\gamma_0=0,\qquad
M=0.
$$

The solver reports the physical residuals and rejects solutions that do not
meet the requested normalized tolerance. Aerodynamic curves and their declared
speed envelope are never silently extrapolated.

## Linear state-space models

The longitudinal state and input vectors are

$$
x_{lon}=[u,w,q,\theta]^T,\qquad
u_{lon}=[\delta_e,\delta_T]^T.
$$

The $w$-acceleration equation includes $Z_{\dot w}$, the pitch equation
includes $M_{\dot w}$, and the pitch-rate coupling is $V_0+Z_q$.

The lateral state and input vectors are

$$
x_{lat}=[v,p,r,\phi]^T,\qquad
u_{lat}=[\delta_a,\delta_r]^T.
$$

Its first row includes $Y_r-V_0$, and its roll/yaw rows use the
cross-inertia-corrected derivatives. The stability-axis kinematic relation is
$\dot\phi=p+r\tan\gamma_0$.

Both builders return complete continuous-time $A,B,C,D$ matrices with identity
full-state output matrices.

## Scope limitation

These are small-disturbance models about one converged trim point. They are not
a nonlinear six-degree-of-freedom simulation and are not suitable for flight
control deployment without independent aerodynamic validation, uncertainty
analysis, nonlinear simulation, hardware-in-the-loop testing, and flight-test
verification.

## References

- F. O. Smetana, D. C. Summery, and W. D. Johnson,
  [*Riding and Handling Qualities of Light Aircraft: A Review and Analysis*,
  NASA-CR-1975](https://ntrs.nasa.gov/citations/19720018355), 1972. The report
  includes derivations of linearized equations, stability derivatives, modal
  characteristics, and sample calculations.
- W. L. Sherman,
  [*Airplane Stability Calculations with a Card Programmable Pocket
  Calculator*, NASA-TM-78678](https://ntrs.nasa.gov/citations/19780022195),
  1978. This reference explicitly uses SI dimensional stability derivatives
  and stability axes.
