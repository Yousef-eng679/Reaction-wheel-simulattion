/**
 * simulationLoop.ts
 *
 * Fixed-timestep simulation loop orchestrator — the central wiring point for
 * the entire simulation (PRD §5, Phase 4).
 *
 * Data flow (as shown in PRD §5 architecture diagram):
 *
 *   ┌──────────────────────────────────────────────────────┐
 *   │  Each physics step (1 kHz internal rate):            │
 *   │                                                      │
 *   │  trueState ──► SensorModel.sample()                  │
 *   │                    │                                 │
 *   │                    ▼  (SensorReading — noisy, rate-  │
 *   │              Estimator.update()   limited)           │
 *   │                    │                                 │
 *   │                    ▼  (estimatedAngle — NEVER        │
 *   │           PidController.update()  trueState — R2.1)  │
 *   │                    │                                 │
 *   │                    ▼  (controlOutput [N·m])          │
 *   │           MotorModel.update(dutyCycle)               │
 *   │                    │                                 │
 *   │                    ▼  (actualTorqueNm)               │
 *   │           physicsEngine.integrate()                  │
 *   │                    │                                 │
 *   │                    ▼  (new trueState)  ──────────────┤
 *   └──────────────────────────────────────────────────────┘
 *
 * R2.1 — The controller NEVER receives trueState. Only the estimator output
 *         (a scalar estimatedAngle) is passed to pidController.update().
 * R2.3 — The UI layer only calls this class's public API. It must never import
 *         physicsEngine, pidController, motorModel, sensorModel, or estimator directly.
 * R2.5 — Physics stepping is driven by a fixed-timestep accumulator. The external
 *         caller provides realDtMs; physics always steps at PHYSICS_DT_S internally.
 * R1.2 — Physics integration uses RK4 (via physicsEngine.integrate()).
 */

import { integrate } from '../core/physics/physicsEngine.ts';
import { createZeroState } from '../core/physics/rigidBodyState.ts';
import type { RigidBodyState, PhysicsParams } from '../core/physics/rigidBodyState.ts';
import { DefaultMotorModel } from '../core/actuator/motorModel.ts';
import type { MotorParams } from '../core/actuator/motorModel.ts';
import { DefaultSensorModel } from '../core/sensor/sensorModel.ts';
import type { SensorParams, SensorReading } from '../core/sensor/sensorModel.ts';
import { ComplementaryFilter } from '../core/sensor/estimator.ts';
import type { ComplementaryFilterParams } from '../core/sensor/estimator.ts';
import { PidController } from '../core/control/pidController.ts';
import type { PidParams } from '../core/control/pidController.ts';
import { PHYSICS_DT_S } from '../core/physics/constants.ts';

export { type RigidBodyState };

// ---------------------------------------------------------------------------
// Public interfaces (re-exported for use by scenarioRunner and UI)
// ---------------------------------------------------------------------------

/**
 * Full configuration bag for constructing a SimulationLoop.
 * Bundles all sub-module params into one object for clean construction.
 */
export interface SimulationConfig {
  physicsParams: PhysicsParams;
  motorParams: MotorParams;
  sensorParams: SensorParams;
  estimatorParams: ComplementaryFilterParams;
  pidParams: PidParams;

  /**
   * Initial rigid-body state (angle and velocity of body + wheel) [rad, rad/s].
   * Defaults to zero if omitted.
   */
  initialState?: RigidBodyState;

  /** Initial target body angle [rad]. Defaults to 0. */
  initialSetpoint?: number;
}

/**
 * Snapshot of all observable simulation data for one instant in time.
 * This is what the UI and telemetry logger read from the simulation loop.
 * All fields are read-only — the UI must never mutate them.
 */
export interface SimSnapshot {
  /** Elapsed simulation time [s] since last reset. */
  simTimeSec: number;

  /** Ground-truth rigid-body state [rad, rad/s]. Read-only — never pass to controller. */
  trueState: RigidBodyState;

  /** Latest sensor reading (noisy, rate-limited by SensorModel's zero-order hold). */
  sensorReading: SensorReading;

  /** Estimator's current angle estimate [rad]. This is what the controller sees. */
  estimatedAngle: number;

  /** Last control output after clamping [N·m]. What the motor actually received. */
  controlOutput: number;

  /**
   * Last raw (pre-clamp) PID output [N·m].
   * Values outside [outputMin, outputMax] indicate the controller is saturated.
   * Useful for visualizing saturation in Phase 5 telemetry charts.
   */
  rawControlOutput: number;

  /** Current target setpoint [rad]. */
  setpoint: number;

  /** Control error = setpoint − estimatedAngle [rad]. */
  error: number;
}

// ---------------------------------------------------------------------------
// SimulationLoop
// ---------------------------------------------------------------------------

/**
 * SimulationLoop — the central orchestrator.
 *
 * Owns one instance of each sub-module. External callers (UI, tests) interact
 * only through this class's public API — they never hold references to core modules.
 *
 * Fixed-timestep accumulator pattern (R2.5):
 *   The external caller drives time via tick(realDtMs). Internally, the loop
 *   accumulates the elapsed real time and fires as many PHYSICS_DT_S-sized physics
 *   steps as the accumulator allows. This decouples physics rate from render rate.
 *
 * R6.1 physics-integrity check points:
 *   1. R1.1: ω₂ emerges from RK4 integration of torque equations — never from
 *      algebraic substitution. (physicsEngine.integrate() handles this.)
 *   2. R2.1: The only value passed to pidController.update() is
 *      `this._estimator.getEstimate()` — a scalar. No reference to `this._trueState`
 *      is passed into or after the controller call. Search for `trueState` in
 *      this file to verify: it only appears as the physics output and sensor input.
 */
export class SimulationLoop {
  // Sub-module instances (R2.3: UI never gets references to these)
  private readonly _motorModel: DefaultMotorModel;
  private readonly _sensorModel: DefaultSensorModel;
  private readonly _estimator: ComplementaryFilter;
  private readonly _pidController: PidController;

  // Physics params (needed by integrate() each step)
  private readonly _physicsParams: PhysicsParams;

  // Mutable simulation state
  private _trueState: RigidBodyState;
  private _setpoint: number;
  private _simTimeSec: number = 0;
  private _accumulator: number = 0;              // residual real time [s] not yet stepped
  private _paused: boolean = false;

  // Ephemeral per-step values (held for snapshot reads between steps)
  private _lastSensorReading: SensorReading;
  private _lastEstimatedAngle: number = 0;
  private _lastControlOutput: number = 0;        // clamped
  private _lastRawControlOutput: number = 0;     // pre-clamp
  private _lastHeldDutyCycle: number = 0;        // held between control ticks

  // Sensor / control tick scheduling
  private readonly _sensorDt: number;            // [s] period between sensor ticks
  private readonly _stepsPerSensorTick: number;  // physics steps per sensor tick
  private _stepsSinceLastSensorTick: number = 0;

  // One-shot disturbance: applied to the body for exactly one physics step
  private _pendingDisturbanceNm: number = 0;

  // The initial config snapshot — used by reset() to restore original conditions
  private readonly _initialConfig: SimulationConfig;

  constructor(config: SimulationConfig) {
    this._initialConfig = config;
    this._physicsParams = config.physicsParams;

    // Instantiate all sub-modules
    this._motorModel    = new DefaultMotorModel(config.motorParams, config.physicsParams);
    this._sensorModel   = new DefaultSensorModel(config.sensorParams);
    this._estimator     = new ComplementaryFilter(config.estimatorParams);
    this._pidController = new PidController(config.pidParams);

    // Sensor scheduling
    this._sensorDt           = 1 / config.sensorParams.sampleRateHz;
    this._stepsPerSensorTick = Math.round(this._sensorDt / PHYSICS_DT_S);

    // Initial state
    this._trueState = config.initialState
      ? { ...config.initialState }
      : createZeroState();
    this._setpoint = config.initialSetpoint ?? 0;

    // Initial sensor reading (zeros — gets overwritten on first physics step)
    this._lastSensorReading = { gyroOmega2: 0, accelAngleEstimate: 0 };
  }

  // ---------------------------------------------------------------------------
  // Control commands (UI → Sim)
  // ---------------------------------------------------------------------------

  /**
   * Advances the simulation by a real-time delta.
   *
   * Accumulator pattern (R2.5):
   *   accumulator += realDtSec
   *   while accumulator >= PHYSICS_DT_S:
   *     do one physics step
   *     accumulator -= PHYSICS_DT_S
   *
   * @param realDtMs  Real elapsed time since last call [milliseconds].
   *                  Clamped to 100 ms max to prevent spiral-of-death on tab-resume.
   */
  tick(realDtMs: number): void {
    if (this._paused) return;

    // Clamp to prevent death-spiral if tab was backgrounded
    const clampedDtMs = Math.min(realDtMs, 100);
    this._accumulator += clampedDtMs / 1000; // convert to seconds

    while (this._accumulator >= PHYSICS_DT_S) {
      this._stepPhysics();
      this._accumulator -= PHYSICS_DT_S;
    }
  }

  /** Sets the target body angle setpoint [rad]. Applied on the next control tick. */
  setSetpoint(targetAngleRad: number): void {
    this._setpoint = targetAngleRad;
  }

  /**
   * Live-updates PID gains. Applied on the next control tick (no restart needed).
   * Integral and derivative memory are preserved — only the coefficients change.
   */
  setGains(kp: number, ki: number, kd: number): void {
    this._pidController.setGains(kp, ki, kd);
  }

  /**
   * Queues an instantaneous disturbance torque [N·m] applied to the body for
   * exactly one physics step. Simulates a "kick" button in the UI.
   * Multiple calls before the next tick() accumulate (sum, not overwrite).
   */
  applyDisturbanceKick(torqueNm: number): void {
    this._pendingDisturbanceNm += torqueNm;
  }

  /** Pauses/unpauses the simulation. When paused, tick() is a no-op. */
  setPaused(paused: boolean): void {
    this._paused = paused;
  }

  /**
   * Resets the simulation to its original construction-time conditions.
   * Zeroes state, clears all integrators, resets sub-modules.
   * Gains and config are preserved — this is a state reset, not a construction.
   */
  reset(): void {
    this._trueState = this._initialConfig.initialState
      ? { ...this._initialConfig.initialState }
      : createZeroState();
    this._setpoint         = this._initialConfig.initialSetpoint ?? 0;
    this._simTimeSec       = 0;
    this._accumulator      = 0;
    this._lastHeldDutyCycle = 0;
    this._lastControlOutput = 0;
    this._lastRawControlOutput = 0;
    this._lastEstimatedAngle = 0;
    this._pendingDisturbanceNm = 0;
    this._stepsSinceLastSensorTick = 0;
    this._lastSensorReading = { gyroOmega2: 0, accelAngleEstimate: 0 };

    // Reset all sub-module internal state
    this._motorModel.reset();
    this._sensorModel.reset();
    this._estimator.reset();
    this._pidController.reset();
  }

  // ---------------------------------------------------------------------------
  // Read-only getters (Sim → UI)
  // R2.3: These are the ONLY data paths from the sim to the UI layer.
  // ---------------------------------------------------------------------------

  /**
   * Returns a full snapshot of the current simulation state.
   * Primary data source for the UI renderer and telemetry logger.
   */
  getSnapshot(): SimSnapshot {
    return {
      simTimeSec:       this._simTimeSec,
      trueState:        { ...this._trueState },  // shallow copy — prevents UI mutation
      sensorReading:    { ...this._lastSensorReading },
      estimatedAngle:   this._lastEstimatedAngle,
      controlOutput:    this._lastControlOutput,
      rawControlOutput: this._lastRawControlOutput,
      setpoint:         this._setpoint,
      error:            this._setpoint - this._lastEstimatedAngle,
    };
  }

  /** Returns whether the simulation is currently paused. */
  isPaused(): boolean {
    return this._paused;
  }

  /** Returns elapsed simulation time [s] since last reset. */
  getSimTimeSec(): number {
    return this._simTimeSec;
  }

  // ---------------------------------------------------------------------------
  // Private: one fixed-timestep physics step
  // ---------------------------------------------------------------------------

  /**
   * Executes one PHYSICS_DT_S-sized simulation step.
   *
   * Strict ordering (matches PRD §5 data-flow diagram):
   *   1. Sensor samples trueState (at sensor rate — zero-order hold between ticks)
   *   2. Estimator updates (at sensor rate)
   *   3. PID controller update (at sensor rate) — receives ONLY estimator output
   *   4. Motor model update (every physics step, held duty cycle between control ticks)
   *   5. Physics integration (RK4, every physics step)
   *   6. Advance clocks
   *
   * R2.1 compliance: search for `_trueState` below. It is passed to sensor.sample()
   * (the sensor reads ground truth — this is correct, it's how sensors work).
   * It is NEVER passed to _pidController.update(). The controller only receives
   * `this._estimator.getEstimate()`.
   */
  private _stepPhysics(): void {
    const dt = PHYSICS_DT_S;

    // --- Step 1: Sensor sample ---
    // The sensor reads trueState — that's the physics definition of a sensor.
    // The KEY constraint (R2.1) is that this noisy reading goes through the estimator
    // before reaching the controller. trueState never reaches the controller.
    this._lastSensorReading = this._sensorModel.sample(this._trueState, dt);

    // --- Steps 2 & 3: Estimator + controller (at sensor rate only) ---
    if (this._stepsSinceLastSensorTick === 0) {
      // Estimator update: fuses noisy sensor data into angle estimate
      this._estimator.update(this._lastSensorReading, this._sensorDt);
      this._lastEstimatedAngle = this._estimator.getEstimate();

      // PID update: receives ONLY the estimator's angle estimate (R2.1)
      // Never receives this._trueState — see R6.1 / R2.1 audit note in class doc.
      const pidOut = this._pidController.update(
        this._setpoint,
        this._lastEstimatedAngle,  // ← estimator output only — R2.1 satisfied
        this._sensorDt,
      );

      // Convert PID output [N·m] to motor duty cycle [−1, 1].
      // Sign convention: positive body angle error → negative duty cycle (wheel spins
      // backward → reaction torque pushes body forward). See Phase 3 sign derivation.
      const outputMax = this._initialConfig.pidParams.outputMax;
      this._lastHeldDutyCycle = -(pidOut / outputMax);
      this._lastControlOutput = pidOut;
      this._lastRawControlOutput = this._pidController.getLastRawOutput();
    }
    this._stepsSinceLastSensorTick =
      (this._stepsSinceLastSensorTick + 1) % this._stepsPerSensorTick;

    // --- Step 4: Motor model update (every physics step, held duty cycle) ---
    const actualTorqueNm = this._motorModel.update(this._lastHeldDutyCycle, dt);

    // --- Step 5: Physics integration (RK4 — R1.2) ---
    // Apply any queued one-shot disturbance to the body torque this step.
    const disturbance = this._pendingDisturbanceNm;
    this._pendingDisturbanceNm = 0; // consume
    this._trueState = integrate(
      this._trueState,
      actualTorqueNm,    // motor torque (applied to wheel, reaction to body)
      disturbance,       // direct body disturbance torque (kick)
      dt,
      this._physicsParams,
    );

    // --- Step 6: Advance simulation clock ---
    this._simTimeSec += dt;
  }
}
