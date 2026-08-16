/**
 * simulationLoop.ts
 *
 * Fixed-timestep simulation loop orchestrator.
 *
 * Wires the four decoupled modules in the exact data flow described in PRD §5:
 *   physics → sensor → estimator → controller → actuator → physics (repeat)
 *
 * Key architectural requirements (PRD §5, R2.1, R2.5):
 *   - Physics steps at a fixed internal rate (PHYSICS_DT_S), independent of render FPS.
 *   - The controller NEVER receives trueState directly (R2.1).
 *     Data flow: physics → sensor/estimator → controller.
 *   - External callers (UI) call tick(realDtMs) which accumulates elapsed time and
 *     fires as many fixed physics steps as the accumulator allows.
 *   - UI reads state via read-only getters — it does not directly access any core module.
 *
 * Phase 0: interface and class skeleton only. All method bodies throw 'not implemented'.
 * Phase 4 will wire the full loop.
 */

import type { RigidBodyState, PhysicsParams } from '../core/physics/rigidBodyState.ts';
import type { MotorModel, MotorParams } from '../core/actuator/motorModel.ts';
import type { SensorModel, SensorReading, SensorParams } from '../core/sensor/sensorModel.ts';
import type { Estimator, ComplementaryFilterParams } from '../core/sensor/estimator.ts';
import type { PidController, PidParams } from '../core/control/pidController.ts';

export { type RigidBodyState };

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
 */
export interface SimSnapshot {
  /** Elapsed simulation time [s] since last reset. */
  simTimeSec: number;

  /** Ground-truth rigid-body state [rad, rad/s]. */
  trueState: RigidBodyState;

  /** Latest sensor reading (noisy, rate-limited). */
  sensorReading: SensorReading;

  /** Estimator's current angle estimate [rad]. */
  estimatedAngle: number;

  /** Last control output returned by the PID controller [N·m]. */
  controlOutput: number;

  /** Last raw (pre-clamp) PID output [N·m]. Useful for visualizing saturation. */
  rawControlOutput: number;

  /** Current target setpoint [rad]. */
  setpoint: number;

  /** Control error = setpoint − estimatedAngle [rad]. */
  error: number;
}

/**
 * SimulationLoop — the central orchestrator.
 *
 * Owns one instance of each sub-module. External callers (UI, tests) interact
 * only through this class's public API — never through sub-module references.
 *
 * R2.3 — The UI layer must never import physicsEngine, pidController, motorModel,
 *         sensorModel, or estimator directly. All communication goes through here.
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 4.
 */
export class SimulationLoop {
  private readonly config: SimulationConfig;
  // These will be instantiated in Phase 4:
  private _motorModel!: MotorModel;
  private _sensorModel!: SensorModel;
  private _estimator!: Estimator;
  private _pidController!: PidController;

  constructor(config: SimulationConfig) {
    this.config = config;
    void this.config;
  }

  // ---------------------------------------------------------------------------
  // Control commands (UI → Sim)
  // ---------------------------------------------------------------------------

  /**
   * Advances the simulation by a real-time delta.
   * Internally fires as many fixed physics steps as the accumulator allows.
   *
   * @param realDtMs  Real elapsed time since last call [milliseconds].
   *                  The physics integrates at PHYSICS_DT_S internally regardless.
   *
   * @throws Error('not implemented')
   */
  tick(_realDtMs: number): void {
    throw new Error('not implemented');
  }

  /**
   * Sets the target body angle setpoint [rad].
   * Applied immediately on the next control tick.
   *
   * @throws Error('not implemented')
   */
  setSetpoint(_targetAngleRad: number): void {
    throw new Error('not implemented');
  }

  /**
   * Live-updates PID gains. Applied on the next control tick (no restart needed).
   *
   * @throws Error('not implemented')
   */
  setGains(_kp: number, _ki: number, _kd: number): void {
    throw new Error('not implemented');
  }

  /**
   * Injects an instantaneous disturbance torque [N·m] applied to the body for one physics step.
   * Simulates a "kick" button in the UI.
   *
   * @throws Error('not implemented')
   */
  applyDisturbanceKick(_torqueNm: number): void {
    throw new Error('not implemented');
  }

  /**
   * Pauses/unpauses the simulation.
   * When paused, tick() is a no-op.
   *
   * @throws Error('not implemented')
   */
  setPaused(_paused: boolean): void {
    throw new Error('not implemented');
  }

  /**
   * Resets the simulation to initial conditions (zeroes state, clears integrators).
   *
   * @throws Error('not implemented')
   */
  reset(): void {
    throw new Error('not implemented');
  }

  // ---------------------------------------------------------------------------
  // Read-only getters (Sim → UI, read-only telemetry)
  // ---------------------------------------------------------------------------

  /**
   * Returns a full snapshot of the current simulation state.
   * This is the primary data source for the UI renderer and telemetry logger.
   * Returns read-only data — UI must not mutate any fields.
   *
   * @throws Error('not implemented')
   */
  getSnapshot(): SimSnapshot {
    throw new Error('not implemented');
  }

  /** Returns whether the simulation is currently paused. */
  isPaused(): boolean {
    throw new Error('not implemented');
  }

  /** Returns elapsed simulation time [s] since last reset. */
  getSimTimeSec(): number {
    throw new Error('not implemented');
  }
}
