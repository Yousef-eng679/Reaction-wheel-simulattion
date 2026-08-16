# Reaction Wheel Attitude Control Simulator

A fully integrated, physically-accurate, software-only simulation of a single-axis reaction wheel attitude control system. Built as a precursor to a physical hardware build using a repurposed HDD spindle motor and an ESP32.

> **Phase 0 — Scaffold complete.** UI and physics are not yet implemented.
> See `IMPLEMENTATION_PLAN.md` in the repository root for the full phase plan.

---

## How to run

```bash
# Install dependencies (first time only)
npm install

# Start the dev server (http://localhost:5173)
npm run dev

# Run the automated test suite
npm test

# Run tests in watch mode (re-runs on file change)
npm run test:watch
```

---

## Project structure

```
src/
  core/
    physics/
      rigidBodyState.ts   — State type: theta1, omega1, theta2, omega2
      physicsEngine.ts    — integrate(state, torque, dt) → State using RK4
      constants.ts        — Named physical constants, SI units only
    actuator/
      motorModel.ts       — MotorModel interface + first-order-lag implementation
    sensor/
      sensorModel.ts      — SensorModel interface + MPU6050-equivalent simulation
      estimator.ts        — Estimator interface + ComplementaryFilter implementation
    control/
      pidController.ts    — Pure, portable PID controller (no framework dependencies)
  sim/
    simulationLoop.ts     — Fixed-timestep accumulator orchestrator
    scenarioRunner.ts     — Headless runner for automated scenario tests
  ui/
    main.ts               — App entry point (wired in Phase 5)
    canvasRenderer.ts     — 2D top-down Canvas visualization
    telemetryChart.ts     — Scrolling strip charts
    controlPanel.ts       — Live parameter controls
    exportImport.ts       — CSV/JSON export + JSON import
  tests/
    physics.conservation.test.ts  — Angular momentum conservation invariant
    physics.integration.test.ts   — RK4 accuracy vs. analytical solution
    control.stepResponse.test.ts  — Step response settling and overshoot
    control.saturation.test.ts    — Graceful saturation (no divergence)
    control.noiseRobustness.test.ts — Convergence under realistic sensor noise
```

---

## Configurable parameters and their meanings

All internal physics computation uses SI units exclusively. Display conversion to degrees/RPM happens only in the UI layer.

| Parameter | Symbol | Unit | Description |
|---|---|---|---|
| Wheel inertia | I₁ | kg·m² | Moment of inertia of the reaction wheel (rotor) |
| Body inertia | I₂ | kg·m² | Moment of inertia of the spacecraft body |
| Max motor RPM | — | rev/min | Maximum wheel speed the motor can reach |
| Max motor torque | τ_max | N·m | Maximum torque the motor can produce |
| Motor time constant | τ_lag | s | First-order lag response time of the motor |
| Wheel friction coeff | b₁ | N·m·s/rad | Bearing/air friction on the wheel |
| Body friction coeff | b₂ | N·m·s/rad | Bearing/air friction on the body |
| Gyro noise σ | σ_gyro | rad/s | Gaussian noise std dev on gyroscope reading |
| Accel angle noise σ | σ_accel | rad | Gaussian noise std dev on accel-derived angle |
| Gyro bias drift rate | — | rad/s/s | Rate of random walk in gyroscope bias |
| Sensor sample rate | — | Hz | IMU sample rate (independent of physics rate) |
| Complementary α | α | — | Blend: gyro (α) vs. accelerometer (1−α) |
| Physics dt | dt | s | Fixed internal integration timestep (1000 Hz) |
| Kp | Kp | N·m/rad | PID proportional gain |
| Ki | Ki | N·m/(rad·s) | PID integral gain |
| Kd | Kd | N·m·s/rad | PID derivative gain (applied to measurement) |

---

## Parameter status: measured vs. placeholder

> **⚠️ All parameter default values are PLACEHOLDER ESTIMATES.** None have been measured from the real hardware. They are representative of a repurposed HDD spindle motor system but must be replaced with measured values during the physical-build phase.

| Parameter | Status | Notes |
|---|---|---|
| I₁ (wheel inertia) | **Placeholder** | Representative of 3.5" HDD rotor |
| I₂ (body inertia) | **Placeholder** | Representative of small demo platform |
| Max motor RPM | **Placeholder** | 5400 RPM typical HDD spindle range |
| Max motor torque | **Placeholder** | Requires dyno/stall-torque measurement |
| Motor time constant | **Placeholder** | Requires step-response characterization |
| Friction coefficients | **Placeholder** | Requires spin-down curve measurement |
| Gyro noise σ | **Placeholder** | Representative MPU6050 spec, not measured |
| Accel angle noise σ | **Placeholder** | Representative MPU6050 spec, not measured |
| Gyro bias drift rate | **Placeholder** | Representative MPU6050 in-run stability |
| PID gains (Kp, Ki, Kd) | **Placeholder** | Tuned against placeholder inertias; will need re-tuning |

---

## Architecture integrity constraints

The `.roles` file in this directory is the standing constraint document for all agent missions on this project. Key rules:

- **R1.1** — `ω₂` must always emerge from integrating torque equations, never from algebraic shortcut `ω₂ = -(I₁/I₂)·ω₁`.
- **R1.2** — Physics integration uses RK4. Euler integration is forbidden.
- **R1.3** — All internal computation in SI units; display conversion only at UI boundary.
- **R2.1** — The PID controller never receives ground-truth state; only sensor/estimator output.
- **R2.4** — Sensor noise uses a seeded PRNG. `Math.random()` is forbidden in `core/sensor/`.
- **R2.5** — Physics steps at a fixed internal rate; render FPS does not drive physics `dt`.

---

## Technology stack

- **Language**: TypeScript
- **Build/dev server**: Vite
- **Rendering**: HTML5 Canvas 2D (hand-rolled — no heavy charting/animation library)
- **Testing**: Vitest
- **No backend** — fully client-side static app

---

*See `PRD.md` and `IMPLEMENTATION_PLAN.md` in the repository root for full requirements and phase-by-phase execution plan.*
