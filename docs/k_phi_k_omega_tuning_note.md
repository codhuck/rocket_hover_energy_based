# Short Note on `k_phi` and `k_omega` Tuning

## 1. Where the coefficients are used

The plant equation for the angular motion is:

```text
phi_ddot = -(alpha F_max l_cp / J) sin(delta)
```

The controller computes:

```text
e_phi = wrap(phi - phi_target)
sin(delta) = (J / (alpha F_max l_cp)) (k_phi e_phi + k_omega omega)
```

Substituting the controller into the plant gives the closed-loop angular equation:

```text
phi_ddot = -(k_phi e_phi + k_omega omega)
```

Since `phi_target` is constant in all current runs:

```text
e_phi_dot = omega
```

So the error dynamics become:

```text
e_phi_ddot + k_omega e_phi_dot + k_phi e_phi = 0
```

This is the equation we are effectively tuning.

---

## 2. Meaning of the variables

- `phi`: current rocket pitch angle
- `phi_target`: desired pitch angle
- `e_phi`: angle error
- `omega`: angular rate, `omega = phi_dot`
- `delta`: nozzle gimbal angle
- `alpha`: thrust scale factor
- `F_max`: maximum thrust
- `l_cp`: thrust moment arm
- `J`: pitch inertia
- `k_phi`: angle-error gain
- `k_omega`: angular-rate damping gain

---

## 3. What `k_phi` and `k_omega` do

- `k_phi` controls how strongly the rocket is pushed back toward the target angle.
- `k_omega` controls how strongly the rotation is damped.

In simple terms:

- larger `k_phi` -> faster correction, larger gimbal demand, more chance of overshoot
- larger `k_omega` -> less oscillation, less overshoot, more damping

So:

- `k_phi` is the "how hard do we pull back?" coefficient
- `k_omega` is the "how hard do we brake the rotation?" coefficient

---

## 4. Values we tested

Only `k_phi` and `k_omega` were changed. The physics and initial condition stayed the same.

| Preset | `k_phi` | `k_omega` |
|---|---:|---:|
| Default | 18.0 | 7.0 |
| Soft | 6.0 | 4.0 |
| Balanced | 14.0 | 7.0 |
| Fast | 24.0 | 7.0 |
| Springy | 18.0 | 2.0 |

---

## 5. Angular-error stabilization criterion

For the attitude-only tuning study, the stabilization criterion is:

```text
|e_phi(t)| = |wrap(phi(t) - phi_target)| < delta_phi
```

with:

```text
phi(0) = 20 deg
phi_target = 0 deg
delta_phi = 0.1 rad
```

The settling time is defined as the first time after which this inequality remains true for the rest of the simulation.

<table>
  <thead>
    <tr>
      <th>Preset</th>
      <th>k_phi</th>
      <th>k_omega</th>
      <th>phi(0) [deg]</th>
      <th>phi_target [deg]</th>
      <th>delta_phi [rad]</th>
      <th>Settling time [s]</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Default</td>
      <td>18.0</td>
      <td>7.0</td>
      <td>20.0</td>
      <td>0.0</td>
      <td>0.1</td>
      <td>0.4931</td>
    </tr>
    <tr>
      <td>Soft</td>
      <td>6.0</td>
      <td>4.0</td>
      <td>20.0</td>
      <td>0.0</td>
      <td>0.1</td>
      <td>0.8125</td>
    </tr>
    <tr>
      <td>Balanced</td>
      <td>14.0</td>
      <td>7.0</td>
      <td>20.0</td>
      <td>0.0</td>
      <td>0.1</td>
      <td>0.6111</td>
    </tr>
    <tr>
      <td>Fast</td>
      <td>24.0</td>
      <td>7.0</td>
      <td>20.0</td>
      <td>0.0</td>
      <td>0.1</td>
      <td>0.3889</td>
    </tr>
    <tr>
      <td>Springy</td>
      <td>18.0</td>
      <td>2.0</td>
      <td>20.0</td>
      <td>0.0</td>
      <td>0.1</td>
      <td>0.9792</td>
    </tr>
  </tbody>
</table>

---

## 6. Results we obtained

Metrics:

- settle time: first time after which both `|phi| < 1 deg` and `|omega| < 1 deg/s`
- overshoot: how far the angle crossed past the target
- max `|delta|`: peak nozzle deflection
- peak `|omega|`: peak angular rate

<table>
  <thead>
    <tr>
      <th>Preset</th>
      <th>Settle time [s]</th>
      <th>Overshoot [deg]</th>
      <th>Max |delta| [deg]</th>
      <th>Peak |omega| [deg/s]</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Default</td>
      <td>1.132</td>
      <td>0.205</td>
      <td>3.764</td>
      <td>35.483</td>
    </tr>
    <tr>
      <td>Soft</td>
      <td>1.792</td>
      <td>0.239</td>
      <td>1.089</td>
      <td>20.847</td>
    </tr>
    <tr>
      <td>Balanced</td>
      <td>1.479</td>
      <td>0.005</td>
      <td>2.773</td>
      <td>28.954</td>
    </tr>
    <tr>
      <td>Fast</td>
      <td>1.396</td>
      <td>0.812</td>
      <td>5.254</td>
      <td>44.570</td>
    </tr>
    <tr>
      <td>Springy</td>
      <td>4.306</td>
      <td>9.377</td>
      <td>4.261</td>
      <td>61.693</td>
    </tr>
  </tbody>
</table>

---

## 7. Why the results look like this

### Default: `k_phi = 18`, `k_omega = 7`

- Fast and clean response
- Small overshoot
- Moderate nozzle motion

Why:
- `k_phi` is high enough to correct quickly
- `k_omega` is also high, so the motion is strongly damped

### Soft: `k_phi = 6`, `k_omega = 4`

- Slowest clean response among the damped cases
- Small nozzle motion
- Low peak angular rate

Why:
- low `k_phi` means weak restoring action
- the rocket comes back gently, not aggressively

### Balanced: `k_phi = 14`, `k_omega = 7`

- Very smooth response
- Almost zero overshoot
- Smaller nozzle motion than default

Why:
- `k_phi` is a bit lower than default, so the pullback is less aggressive
- `k_omega` stays high, so damping remains strong

### Fast: `k_phi = 24`, `k_omega = 7`

- Faster, more aggressive response
- Larger nozzle motion
- Higher peak angular rate
- More overshoot than default

Why:
- higher `k_phi` makes the controller push much harder
- `k_omega` is still high, so it stays stable, but the stronger correction creates a sharper transient

### Springy: `k_phi = 18`, `k_omega = 2`

- Largest overshoot
- Largest peak angular rate
- Longest settling time
- Visibly oscillatory response

Why:
- `k_phi` is still strong, so the system pushes hard
- `k_omega` is too low, so there is not enough damping to stop the motion cleanly

---

## 8. Main conclusions

1. `k_phi` mainly sets correction strength.
2. `k_omega` mainly sets damping.
3. High `k_phi` with low `k_omega` gives a springy, underdamped response.
4. Low `k_phi` with moderate `k_omega` gives a soft but slower response.
5. For this project:
   - `balanced` is the smoothest clean tuning
   - `default` is the best fast clean tuning
   - `fast` is useful when a more aggressive response is desired
   - `springy` is the clearest example of insufficient damping

---

## 9. What we should change and what we should not

Safe to change:

- `k_phi`
- `k_omega`
- initial angle
- initial angular rate
- simulation duration

Better not to change:

- the sign of the plant equation
- the sign of the control law
- the definition of `e_phi = wrap(phi - phi_target)`
- physical parameters, unless the goal is to study a different rocket
