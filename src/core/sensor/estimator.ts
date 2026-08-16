/**
 * estimator.ts
 *
 * State estimator: produces the estimated body angle that the PID controller uses.
 *
 * The estimator sits between the raw sensor readings (SensorModel) and the controller.
 * The controller MUST NEVER receive ground-truth state (R2.1) — only what the estimator
 * outputs.
 *
 * Default implementation: Complementary Filter (matching PRD §6.3 and the physics
 * document's §3.2 plan for the real ESP32 firmware):
 *   θ_estimated = α · (θ_prev + ω_gyro · dt) + (1 − α) · θ_accel
 *
 * α close to 1.0 → trusts gyro (fast, but drifts).
 * α close to 0.0 → trusts accelerometer (no drift, but noisy and acceleration-sensitive).
 *
 * PRD §6.5: the estimator is behind an interface so a future Kalman filter or alternative
 * can be swapped in without touching physics or actuator modules.
 *
 * Phase 0: interfaces and signatures only. Bodies throw 'not implemented'.
 * Phase 2 will provide the full ComplementaryFilter implementation.
 */

import type { SensorReading } from './sensorModel.ts';

/**
 * Estimator interface — the public contract for all state estimator implementations.
 * The simulation loop calls update() each sensor tick and passes the result to the controller.
 */
export interface Estimator {
  /**
   * Processes the latest sensor reading and returns the current estimated body angle.
   *
   * @param reading  Raw sensor reading from the SensorModel (gyro + accel).
   * @param dt       Elapsed time since last estimator update [s].
   * @returns        Estimated body angle [rad] — the value the controller will act on.
   *                 This is never ground truth (R2.1).
   */
  update(reading: SensorReading, dt: number): number;

  /** Resets estimator internal state (for clean simulation reset). */
  reset(): void;

  /** Returns the current angle estimate [rad] without advancing time. */
  getEstimate(): number;
}

/**
 * Complementary filter configuration.
 */
export interface ComplementaryFilterParams {
  /**
   * Blend coefficient α [dimensionless, 0–1].
   * θ_est = α · (θ_prev + gyro·dt) + (1−α) · accel_angle
   * PLACEHOLDER — 0.98 is a common starting point (R1.4).
   */
  alpha: number;
}

/**
 * ComplementaryFilter: default Estimator implementation.
 * Fuses gyroscope (rate integration) and accelerometer (absolute tilt) readings.
 * Directly mirrors the complementary filter planned for the ESP32 firmware (PRD §6.3).
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 2.
 */
export class ComplementaryFilter implements Estimator {
  private readonly params: ComplementaryFilterParams;

  constructor(params: ComplementaryFilterParams) {
    this.params = params;
    void this.params;
  }

  update(_reading: SensorReading, _dt: number): number {
    throw new Error('not implemented');
  }

  reset(): void {
    throw new Error('not implemented');
  }

  getEstimate(): number {
    throw new Error('not implemented');
  }
}
