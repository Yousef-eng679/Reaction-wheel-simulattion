# Reaction Wheel Attitude Control Simulator

A physically accurate, real-time, software-only simulation of a single-axis reaction wheel
attitude control system — built as a pre-hardware validation tool for an ESP32 + MPU6050 +
repurposed hard-drive spindle motor project.

**Status:** Phases 0–6 complete. All 36 automated tests passing.

---

## Quick Start

### Run the app

```bash
cd reaction-wheel-sim
npm install
npm run dev
```

Open **http://localhost:5173** (or the port Vite prints) in any modern browser.

### Run the test suite

```bash
cd reaction-wheel-sim
npm test
```

Expected output: `Tests 36 passed (36)` — 8 test files covering physics, actuator, sensor,
estimator, controller, integration, and noise robustness.

---

## What the simulator shows

The simulator models the angular momentum exchange between a small **reaction wheel** (rotor)
and a **spacecraft body** (stator). The reaction wheel is spun up by a motor; by Newton's third
law, the body counter-rotates in proportion to the angular momentum transferred.

A closed-loop PID controller reads the *estimated* body angle (from the noisy IMU model, not
ground truth), computes a torque command, and drives the motor to bring the body to the target
angle.

### Architecture diagram

```
physics (RK4) → sensor model (noisy IMU) → complementary filter
                                                       ↓
                                                 PID controller
                                                       ↓
                                                  motor model (lag/saturation)
                                                       ↓
                                             back to physics engine
```

All layers communicate through `SimulationLoop`'s public API. The UI never imports core modules.

---

## Controls

| Control | Effect |
|---|---|
| **Target angle** slider | Sets body angle setpoint (applied live, no restart) |
| **Kp / Ki / Kd** sliders | PID gain live-update (no restart needed) |
| **I₁ / I₂** sliders | Wheel / body inertia — rebuilds simulation |
| **Wheel/Body friction** sliders | Parasitic damping — rebuilds simulation |
| **Max RPM / Max torque / Lag τ** | Motor constraints — rebuilds simulation |
| **Gyro noise σ / Accel noise σ** | IMU noise level — rebuilds simulation |
| **Filter α** | Complementary filter blend — rebuilds simulation |
| **💥 Kick** | Injects an instantaneous disturbance torque to the body |
| **⏸ Pause / ▶ Resume** | Pauses/resumes physics steps |
| **↺ Reset** | Resets all state and telemetry to t=0 |
| **⬇ CSV** | Downloads all logged telemetry as a CSV file |
| **⬇ Config** | Downloads the current config as a JSON file |
| **⬆ Import** | Loads a previously exported JSON config to restore a scenario |

### Canvas legend

| Colour | Meaning |
|---|---|
| White arm | True body angle (ground truth) |
| Green dashed arm | Estimated angle (what the controller sees) |
| Purple dot / dashed line | Target setpoint |
| Orange arc | Angular error (setpoint − true angle) |
| Red dashed ring | Actuator saturation (motor at maximum torque) |

### Chart channels

| Chart | Signals | Units |
|---|---|---|
| Angle | True θ₂ (purple), Estimated θ₂ (green) | deg |
| ω₂ body | True body angular velocity | rad/s |
| Wheel RPM | True wheel speed | RPM |
| Control output | Clamped output (red), Raw pre-clamp (dim) | N·m |
| Error | setpoint − estimated angle | deg |

---

## Parameters reference

All internal physics and control computation uses **SI units exclusively**:
kilograms, meters, radians, rad/s, N·m, seconds. Conversion to degrees or RPM
happens only at the display layer.

### Inertia parameters

| Symbol | Name | Unit | Default | Status |
|---|---|---|---|---|
| I₁ | Wheel moment of inertia | kg·m² | `1e-4` | ⚠️ **Placeholder** |
| I₂ | Body moment of inertia | kg·m² | `5e-3` | ⚠️ **Placeholder** |

> ⚠️ These are representative estimates for a 3.5" HDD rotor and a small (~200 g, ~0.1 m)
> demo platform. **Replace with values measured from your actual hardware** using a bifilar
> pendulum or other standard method.

### Motor (actuator) parameters

| Symbol | Name | Unit | Default | Status |
|---|---|---|---|---|
| maxRPM | Maximum wheel speed | rev/min | `5400` | ⚠️ **Placeholder** |
| maxTorqueNm | Maximum motor torque | N·m | `0.05` | ⚠️ **Placeholder** |
| τ_motor | First-order lag time constant | s | `0.1` | ⚠️ **Placeholder** |

> ⚠️ All motor parameters are placeholder estimates representative of a typical HDD spindle
> motor. **Replace with values measured via step-response characterisation** of your specific
> motor once the hardware is assembled. The lag constant in particular is highly motor- and
> driver-dependent.

### Friction parameters

| Symbol | Name | Unit | Default | Status |
|---|---|---|---|---|
| b₁ | Wheel bearing friction coefficient | N·m·s/rad | `1e-5` | ⚠️ **Placeholder** |
| b₂ | Body bearing friction coefficient | N·m·s/rad | `1e-5` | ⚠️ **Placeholder** |

> Near-zero defaults allow the simulation to demonstrate near-ideal angular momentum
> conservation. Increasing friction deliberately breaks conservation — this is physically
> correct and pedagogically useful (you can see how the total angular momentum drifts).

### Sensor (IMU) parameters

| Symbol | Name | Unit | Default | Status |
|---|---|---|---|---|
| σ_gyro | Gyroscope noise standard deviation | rad/s | `0.005` | ⚠️ **Placeholder** |
| σ_accel | Accelerometer angle noise std dev | rad | `0.02` | ⚠️ **Placeholder** |
| bias drift | Gyro bias drift rate | rad/s per √s | `1e-4` | ⚠️ **Placeholder** |
| sampleRateHz | IMU sample rate | Hz | `200` | Representative |

> ⚠️ These noise values are representative of an MPU6050 at default bandwidth settings.
> The true values depend on your specific module, orientation, and mounting. Characterise
> your sensor using a stationary Allan variance measurement before tuning the filter.

### Complementary filter

| Symbol | Name | Range | Default | Status |
|---|---|---|---|---|
| α | Gyro/accel blend coefficient | [0, 1] | `0.98` | Tunable |

> α close to 1.0 trusts the gyro more (low noise, short-term accurate, drifts long-term).
> α close to 0.0 trusts the accelerometer more (no drift, but polluted by vibration and
> centripetal acceleration during rotation). 0.98 is a common real-firmware starting point.

### PID gains

| Symbol | Name | Unit | Default | Status |
|---|---|---|---|---|
| Kp | Proportional gain | N·m/rad | `0.5` | ⚠️ **Placeholder** |
| Ki | Integral gain | N·m/(rad·s) | `0.05` | ⚠️ **Placeholder** |
| Kd | Derivative gain | N·m·s/rad | `0.1` | ⚠️ **Placeholder** |

> ⚠️ Default gains are tuned empirically against the placeholder inertia values above.
> **When you replace I₁ and I₂ with measured values, retune the gains.** A good starting
> point: set Ki=0 and Kd=0, increase Kp until you see oscillation, then back off to ~60%
> of that value, add Kd to damp oscillation, then add small Ki to eliminate steady-state error.

---

## Data export

### CSV telemetry export (⬇ CSV)

Downloads all telemetry logged during the current run as a CSV file. Columns (all SI units):

| Column | Description | Unit |
|---|---|---|
| `time_s` | Simulation time | s |
| `true_angle_rad` | True body angle θ₂ | rad |
| `true_omega2_rad_s` | True body angular velocity ω₂ | rad/s |
| `wheel_omega1_rad_s` | True wheel angular velocity ω₁ | rad/s |
| `true_theta1_rad` | True wheel angle θ₁ | rad |
| `estimated_angle_rad` | Estimator output (controller input) | rad |
| `control_output_nm` | Clamped PID output | N·m |
| `raw_control_output_nm` | Pre-clamp PID output (> limits = saturated) | N·m |
| `error_rad` | setpoint − estimatedAngle | rad |
| `setpoint_rad` | Current target body angle | rad |
| `gyro_reading_rad_s` | Noisy gyroscope reading | rad/s |
| `accel_angle_rad` | Accelerometer-derived angle estimate | rad |

### JSON config export/import (⬇ Config / ⬆ Import)

- **Export** saves all current slider values as a JSON file.
- **Import** loads a previously exported file, restores all parameters exactly, and resets
  the simulation with the same sensor seed — enabling fully reproducible sessions.

---

## Automated tests

Run with `npm test`. Eight test files:

| File | What it tests |
|---|---|
| `physics.integration.test.ts` | RK4 integration accuracy, step size independence |
| `physics.conservation.test.ts` | Angular momentum conservation (zero friction + disturbance) |
| `actuator.motorModel.test.ts` | Motor lag, RPM limit, torque saturation |
| `sensor.estimator.test.ts` | Complementary filter convergence, noise rejection |
| `control.stepResponse.test.ts` | 45° step: settle ≤30 s, overshoot ≤20%, real physics engine |
| `control.saturation.test.ts` | System bounded under 10-rotation saturation command |
| `control.noiseRobustness.test.ts` | 30° step with full sensor noise: settle ±5°, no oscillation |
| `integration.fullLoop.test.ts` | Full orchestrated loop, disturbance recovery, R2.1 audit |

### Key invariants the test suite enforces

- **R1.1** — ω₂ emerges from RK4 torque integration, never from `ω₂ = −(I₁/I₂)·ω₁` algebraic substitution.
- **R2.1** — The controller never receives `trueState` directly (automatically asserted in `integration.fullLoop.test.ts`).
- **R3.3** — All sensor noise in tests uses fixed seeds for reproducibility.

---

## Risks and known limitations

- **Gain transfer to hardware:** PID gains tuned in simulation will not transfer 1:1 to the
  real ESP32 due to unmodelled effects (motor back-EMF dynamics, driver deadband, real
  vibration, sensor mounting). The simulation gives qualitative behaviour and order-of-magnitude
  starting values — real-hardware tuning is still required.
- **Placeholder parameters:** As noted throughout, default inertia and motor values are
  estimates. The simulation is physically correct given those values — the correctness of
  results relative to your specific hardware depends on how well the parameters are characterised.
- **Single-axis only:** The simulation models one rotational axis. Extending to 3-axis
  (three reaction wheels, quaternion attitude) would require significant changes to the
  physics and control layers.

---

## Project structure

```
reaction-wheel-sim/
├── src/
│   ├── core/
│   │   ├── physics/
│   │   │   ├── physicsEngine.ts      # RK4 integration of rotational equations of motion
│   │   │   ├── rigidBodyState.ts     # State vector and physics parameter types
│   │   │   └── constants.ts          # All named physical constants (SI units)
│   │   ├── actuator/
│   │   │   └── motorModel.ts         # First-order lag motor with RPM + torque limits
│   │   ├── sensor/
│   │   │   ├── sensorModel.ts        # MPU6050-equivalent: gyro + accel + bias drift
│   │   │   └── estimator.ts          # Complementary filter (gyro + accel fusion)
│   │   └── control/
│   │       └── pidController.ts      # PID with anti-windup + derivative-on-measurement
│   ├── sim/
│   │   ├── simulationLoop.ts         # Fixed-timestep orchestrator (R2.1, R2.5)
│   │   └── scenarioRunner.ts         # Headless runner for tests + UI "run scenario" button
│   ├── ui/
│   │   ├── main.ts                   # App entry point + RAF loop
│   │   ├── canvasRenderer.ts         # 2D canvas: body, wheel, target, error arc
│   │   ├── telemetryChart.ts         # Scrolling strip charts (ring buffer, no library)
│   │   ├── controlPanel.ts           # All sliders, buttons, live readouts
│   │   └── exportImport.ts           # CSV/JSON export + JSON import with validation
│   ├── tests/                        # Vitest test suite (8 files, 36 tests)
│   └── main.ts                       # Vite entry point (re-exports ui/main.ts)
├── docs/
│   ├── PRD.md                        # Product requirements document
│   └── IMPLEMENTATION_PLAN.md        # Phase-by-phase implementation plan
└── index.html                        # App shell + CSS design system
```
