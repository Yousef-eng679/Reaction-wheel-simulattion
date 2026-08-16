/**
 * scenarioRunner.ts
 *
 * Headless scenario runner — runs a SimulationLoop for N seconds and returns
 * the full telemetry array (Phase 4 deliverable).
 *
 * Used by:
 *   1. The test suite (this phase's integration test, Phase 6's noise-robustness test).
 *      Runs at many × real time — 60 s of simulation completes in milliseconds.
 *   2. The UI's "Run test scenario" button (Phase 5): calls runScenario() and plots
 *      the returned telemetry array.
 *
 * R2.3 — This module imports SimulationLoop (the orchestrator) but does NOT import
 *         physicsEngine, pidController, motorModel, sensorModel, or estimator directly.
 *         All physics/control is accessed exclusively through SimulationLoop's public API.
 * R4.3 — No backend, no file I/O. Runs entirely in memory.
 * R3.3 — Deterministic when scenario's sensor seed is fixed.
 */

import { SimulationLoop } from './simulationLoop.ts';
import type { SimulationConfig, SimSnapshot } from './simulationLoop.ts';
import { PHYSICS_DT_S } from '../core/physics/constants.ts';

// ---------------------------------------------------------------------------
// Public interfaces
// ---------------------------------------------------------------------------

/**
 * Configuration for a single named test scenario.
 */
export interface ScenarioConfig {
  /** Human-readable name for this scenario (used in test output and UI labels). */
  name: string;

  /** Full simulation loop configuration (physics params, gains, sensor params, etc.). */
  simConfig: SimulationConfig;

  /** Target body angle setpoint for the scenario [rad]. */
  setpointRad: number;

  /**
   * Optional disturbance torque to inject at a specific simulation time [N·m].
   * Applied to the body for exactly one physics step (a "kick").
   */
  disturbanceTorqueNm?: number;

  /**
   * Simulation time at which the disturbance kick is applied [s].
   * Ignored if disturbanceTorqueNm is not set.
   */
  disturbanceTimeS?: number;

  /** Total simulation duration [s]. */
  durationSec: number;

  /**
   * Telemetry logging interval [s].
   * One SimSnapshot is recorded every loggingIntervalS of simulation time.
   * Use PHYSICS_DT_S for full resolution or a larger value for memory efficiency.
   * Typical: 10 × PHYSICS_DT_S = 10 ms (100 Hz telemetry) captures all dynamics.
   */
  loggingIntervalS: number;
}

/**
 * Result returned by runScenario().
 */
export interface ScenarioResult {
  /** The scenario configuration that produced this result. */
  config: ScenarioConfig;

  /**
   * Time-ordered telemetry snapshots, one per loggingIntervalS.
   * telemetry[0] is the initial state (t = 0).
   * telemetry[telemetry.length-1] is the final state (t ≈ durationSec).
   */
  telemetry: SimSnapshot[];

  /**
   * Total wall-clock time taken to run the scenario [ms].
   * Used for performance monitoring — a 60 s scenario should run in < 1 s wall time.
   */
  wallClockMs: number;
}

// ---------------------------------------------------------------------------
// runScenario
// ---------------------------------------------------------------------------

/**
 * Runs a simulation scenario headlessly for a fixed duration and returns telemetry.
 *
 * Implementation notes:
 *   - Drives the SimulationLoop by calling tick(PHYSICS_DT_MS) on every step,
 *     so the accumulator always fires exactly one physics step per tick() call.
 *     This makes the scenario runner a deterministic, non-real-time driver.
 *   - Logs a snapshot every loggingIntervalS of simulation time.
 *   - Applies the optional disturbance kick at the specified simulation time
 *     by calling SimulationLoop.applyDisturbanceKick() before the tick.
 *   - Deterministic when the sensor seed is fixed (R3.3).
 *
 * @param scenario  Scenario configuration.
 * @returns         ScenarioResult with telemetry array and wall-clock performance info.
 */
export function runScenario(scenario: ScenarioConfig): ScenarioResult {
  const wallStart = performance.now();

  // Create a fresh SimulationLoop for this scenario
  const loop = new SimulationLoop({
    ...scenario.simConfig,
    initialSetpoint: scenario.setpointRad,
  });

  const totalSteps = Math.round(scenario.durationSec / PHYSICS_DT_S);
  const logEverySteps = Math.max(1, Math.round(scenario.loggingIntervalS / PHYSICS_DT_S));

  // Disturbance scheduling
  const hasDisturbance =
    scenario.disturbanceTorqueNm !== undefined &&
    scenario.disturbanceTimeS !== undefined;
  const disturbanceStep = hasDisturbance
    ? Math.round(scenario.disturbanceTimeS! / PHYSICS_DT_S)
    : -1;
  let disturbanceApplied = false;

  const telemetry: SimSnapshot[] = [];

  // Record the initial state before any stepping
  telemetry.push(loop.getSnapshot());

  // Step the simulation one physics step at a time.
  // We pass PHYSICS_DT_MS to tick() so the accumulator fires exactly one step per call.
  const PHYSICS_DT_MS = PHYSICS_DT_S * 1000;

  for (let step = 0; step < totalSteps; step++) {
    // Apply disturbance kick exactly once at the scheduled step
    if (hasDisturbance && !disturbanceApplied && step === disturbanceStep) {
      loop.applyDisturbanceKick(scenario.disturbanceTorqueNm!);
      disturbanceApplied = true;
    }

    loop.tick(PHYSICS_DT_MS);

    // Log a snapshot at the specified interval (skip step 0 — already logged above)
    if ((step + 1) % logEverySteps === 0) {
      telemetry.push(loop.getSnapshot());
    }
  }

  // Always ensure the final state is logged
  const lastLogged = telemetry[telemetry.length - 1];
  if (lastLogged.simTimeSec < loop.getSimTimeSec() - PHYSICS_DT_S * 0.5) {
    telemetry.push(loop.getSnapshot());
  }

  const wallClockMs = performance.now() - wallStart;

  return { config: scenario, telemetry, wallClockMs };
}
