/**
 * scenarioRunner.ts
 *
 * Headless scenario runner — runs a SimulationLoop for N seconds
 * and returns the full telemetry array.
 *
 * Used by:
 *   1. The test suite (physics.conservation.test.ts, control.stepResponse.test.ts, etc.)
 *      for automated scenario testing without needing a browser/canvas.
 *   2. The UI's "Run test scenario" button (Phase 5), which calls this function
 *      and then plots the returned telemetry.
 *
 * R2.3 — This module may import SimulationLoop (the orchestrator) but must not
 *         import physicsEngine, pidController, or other core modules directly.
 *         All physics/control is accessed through SimulationLoop's public API.
 * R4.3 — No backend, no file I/O here. Runs entirely in memory.
 *
 * Phase 0: interface and signature only. Body throws 'not implemented'.
 * Phase 4 will implement the actual runner logic.
 */

import type { SimulationConfig, SimSnapshot } from './simulationLoop.ts';

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
   * Optional disturbance torque to apply at a specific time [N·m].
   * Simulates a "kick" at t = disturbanceTimeS.
   */
  disturbanceTorqueNm?: number;

  /** Simulated time at which the disturbance kick is applied [s]. */
  disturbanceTimeS?: number;

  /** Total duration to run the scenario [s]. */
  durationSec: number;

  /**
   * Telemetry logging interval [s].
   * The runner records one SimSnapshot every loggingIntervalS of sim time.
   * Set to match physics dt for full-resolution logging, or larger for memory efficiency.
   */
  loggingIntervalS: number;
}

/**
 * Result returned by runScenario.
 */
export interface ScenarioResult {
  /** The scenario that was run. */
  config: ScenarioConfig;

  /**
   * Time-ordered telemetry snapshots, one per logging interval.
   * Snapshot[0] is t=0 (initial state), Snapshot[N] is t=durationSec.
   */
  telemetry: SimSnapshot[];

  /** Total wall-clock time taken to run the scenario [ms]. For performance monitoring. */
  wallClockMs: number;
}

/**
 * Runs a simulation scenario headlessly for a fixed duration and returns telemetry.
 *
 * This function is deterministic when the scenario's sensor seed is fixed (R3.3).
 * It runs at simulation speed (not real time) — 10 seconds of simulation may complete
 * in milliseconds of wall time, which is what makes it suitable for automated tests.
 *
 * @param scenario  The scenario configuration (sim params, setpoint, duration, etc.).
 * @returns         ScenarioResult with full telemetry array and performance info.
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 4.
 */
export function runScenario(_scenario: ScenarioConfig): ScenarioResult {
  throw new Error('not implemented');
}
