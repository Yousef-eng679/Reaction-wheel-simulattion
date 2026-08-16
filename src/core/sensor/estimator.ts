/**
 * estimator.ts
 *
 * State estimator: fuses noisy sensor readings into a single angle estimate
 * that the PID controller acts on (PRD §6.3, §6.5).
 *
 * R2.1 — The estimator's output is the ONLY angle value the controller ever
 *         sees. Ground-truth θ₂ must never be passed to the controller directly.
 * R2.2 — No imports from ui/, sim/, DOM, or framework code.
 *
 * Default implementation: Complementary Filter.
 * Mirrors the filter architecture planned for the real ESP32 firmware (PRD §6.3):
 *
 *   θ_est(k) = α · [ θ_est(k−1) + ω_gyro · dt ] + (1 − α) · θ_accel
 *
 *   where:
 *     α          = blend coefficient (close to 1.0, e.g. 0.98)
 *     ω_gyro     = noisy gyro reading [rad/s] from sensor model
 *     θ_accel    = accelerometer-derived angle [rad] from sensor model
 *     dt         = elapsed time since last update [s]
 *
 * The gyro term (α branch) integrates high-frequency motion accurately.
 * The accel term ((1−α) branch) prevents long-term drift because θ_accel is
 * an absolute angle reference that doesn't accumulate bias over time.
 *
 * PRD §6.5 extensibility hook: Estimator interface is separate from the
 * implementation. A Kalman filter or other estimator can be swapped in by
 * implementing the same interface — without touching physics or actuator code.
 */

import type { SensorReading } from './sensorModel.ts';

// ---------------------------------------------------------------------------
// Estimator interface (PRD §6.5 extensibility hook)
// ---------------------------------------------------------------------------

export interface Estimator {
  /**
   * Processes the latest sensor reading and returns the current angle estimate.
   *
   * @param reading  Raw sensor reading (gyro + accel) from SensorModel.
   * @param dt       Elapsed time since last estimator update [s].
   * @returns        Estimated body angle [rad] — what the controller receives.
   *                 Never ground truth (R2.1).
   */
  update(reading: SensorReading, dt: number): number;

  /** Resets estimator internal state for a clean simulation restart. */
  reset(): void;

  /** Returns the current angle estimate [rad] without advancing time. */
  getEstimate(): number;
}

// ---------------------------------------------------------------------------
// ComplementaryFilter
// ---------------------------------------------------------------------------

export interface ComplementaryFilterParams {
  /**
   * Blend coefficient α [dimensionless, 0–1].
   * θ_est = α · (θ_prev + gyro·dt) + (1−α) · accel_angle
   * Recommended: 0.95–0.99. PLACEHOLDER — tune with real sensor noise data (R1.4).
   */
  alpha: number;
}

/**
 * ComplementaryFilter — default Estimator implementation.
 *
 * Fuses gyroscope (rate integration) and accelerometer (absolute angle) readings.
 * Gyro gives accurate short-term tracking; accel corrects long-term drift.
 *
 * Initialisation: starts with θ_est = 0.  On first update(), the (1−α) accel
 * term quickly pulls the estimate toward the true angle if it started wrong,
 * so warm-up is fast for small α deviations from 1.
 *
 * Portability note (PRD §6.4): this class translates almost line-for-line to
 * embedded C++. The only state is a single float (estimate). The update() logic
 * uses no dynamic allocation, no closures, and no JS-specific features.
 */
export class ComplementaryFilter implements Estimator {
  private readonly params: ComplementaryFilterParams;

  /** Current angle estimate [rad]. The controller reads this every control tick. */
  private estimate: number = 0;

  constructor(params: ComplementaryFilterParams) {
    this.params = params;
  }

  /**
   * Complementary filter update.
   *
   * θ_est(k) = α · [ θ_est(k−1) + ω_gyro · dt ] + (1 − α) · θ_accel
   *
   * @param reading  Sensor reading: gyroOmega2 [rad/s] and accelAngleEstimate [rad].
   * @param dt       Elapsed time since last update [s].
   * @returns        Updated body angle estimate [rad].
   */
  update(reading: SensorReading, dt: number): number {
    const { alpha } = this.params;

    // Gyro branch: integrate angular velocity forward from last estimate
    const gyroIntegrated = this.estimate + reading.gyroOmega2 * dt;

    // Blend: trust gyro for short-term changes, accel for long-term reference
    this.estimate = alpha * gyroIntegrated + (1 - alpha) * reading.accelAngleEstimate;

    return this.estimate;
  }

  /**
   * Resets the estimate to zero.
   * Call on simulation reset to start fresh.
   */
  reset(): void {
    this.estimate = 0;
  }

  /** Returns the current estimate [rad] without advancing the filter. */
  getEstimate(): number {
    return this.estimate;
  }
}
