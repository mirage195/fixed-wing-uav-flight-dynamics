# Modal and time-response analysis

Phase 2 analyzes the continuous-time perturbation model

$$
\dot{x}=Ax+Bu,\qquad y=Cx+Du.
$$

All states and controls are perturbations about the Phase 1 trim point. Linear
responses are therefore meaningful only near that operating condition and
within the aerodynamic data envelope.

## Modal metrics

For an oscillatory pole pair

$$
\lambda=\sigma\pm i\omega_d,
$$

the implementation reports

$$
\omega_n=\sqrt{\sigma^2+\omega_d^2},\qquad
\zeta=-\frac{\sigma}{\omega_n},\qquad
T=\frac{2\pi}{\omega_d}.
$$

For a nonzero real part it also reports the envelope time constant. Stable
modes receive approximate 2% settling and half-amplitude times, while unstable
modes receive a doubling time:

$$
\tau=\frac{1}{|\sigma|},\qquad
t_s\approx\frac{4}{-\sigma},\qquad
t_{1/2}=\frac{\ln 2}{-\sigma},\qquad
t_2=\frac{\ln 2}{\sigma}.
$$

Only the applicable quantities are populated. Marginal roots do not receive a
finite time constant. Real second-order pairs are categorized as critically
damped or overdamped from their equivalent natural frequency and damping
ratio; isolated real poles are reported as aperiodic decay or divergence.

## State participation and mode names

If $V$ is the matrix of right eigenvectors and $W=V^{-1}$ contains the left
eigenvectors, the state-$k$, mode-$i$ participation magnitude is

$$
p_{ki}=\left|V_{ki}W_{ik}\right|.
$$

Participation values are normalized to sum to one for each pole. Values for a
complex-conjugate pair are averaged and normalized again. The largest value is
reported as the dominant state. The JSON and CSV reports retain all normalized
values so the label can be independently reviewed.

Conventional mode names are assigned by transparent heuristics:

- With two longitudinal complex pairs, the lower-frequency pair is the
  phugoid and the higher-frequency pair is the short-period mode. A single
  complex pair is distinguished using the combined $u,\theta$ and $w,q$
  participation.
- The lateral complex pair is the Dutch-roll mode. The fastest real pole is
  assigned to roll subsidence and the remaining real pole nearest the origin
  is assigned to the spiral mode.

These rules are suitable for conventional four-state models but are not
universal. Unusual configurations, repeated roots, strongly coupled modes, or
higher-order models require engineering review of the reported eigenvalues and
participation factors. The eigenvector condition number is included to expose
poorly conditioned modal decompositions.

## Time simulations

The simulation module uses the exact continuous-time LTI representation
provided by SciPy for evenly spaced requested samples. Inputs and initial
conditions are constructed by state or input name, preventing silent ordering
errors. The bundled workflow evaluates:

- a longitudinal initial perturbation and elevator step;
- a lateral initial perturbation and rudder step; and
- complete state and output histories in SI units and radians.

The default cases are diagnostic examples, not prescribed certification
maneuvers. A positive pole produces a growing response and is retained in both
the tables and figures.

## Reproducible outputs

The `uav-dynamics` workflow writes a timestamp-free, sorted JSON document; modal and
time-history CSV files; and both PNG and SVG figures. Fixed SVG hashing and
omitted creation dates make equivalent runs byte-stable with the same Python
and dependency versions. The generated `results/` directory is ignored by Git
so reviewed outputs can be generated locally without polluting commits.
