/**
 * sensorModel.ts
 *
 * Sensor model: simulated MPU6050-equivalent IMU (gyroscope + accelerometer).
 *
 * Models what a real MPU6050 delivers (PRD §6.3):
 *   - Gyroscope reading: noisy angular velocity
 *   - Accelerometer reading: tilt-derived angle estimate (noisy, centripetal-polluted)
 *   - Configurable Gaussian noise on both channels
 *   - Configurable slow gyro bias drift (random walk)
 *   - Configurable sample rate, independently slower than physics rate
 *
 * R2.4 — Uses a seedable PRNG for all noise injection. Math.random() is FORBIDDEN
 *         in this module to ensure deterministic, reproducible tests (R3.3).
 * R1.3 — All sensor outputs in SI units (rad, rad/s). Display conversion elsewhere.
 * R1.4 — All noise parameters are named constants, not inline literals.
 *
 * Phase 0: interfaces and signatures only. Bodies throw 'not implemented'.
 * Phase 2 will provide the full implementation.
 */

import type { RigidBodyState } from '../physics/rigidBodyState.ts';

/**
 * A single IMU sample — what the sensor model produces each sensor tick.
 * This is the raw sensor reading before any filtering.
 */
export interface SensorReading {
  /**
   * Gyroscope angular velocity measurement of the body [rad/s].
   * True ω₂ + Gaussian noise + accumulated bias drift.
   */
  gyroOmega2: number;

  /**
   * Accelerometer-derived body angle estimate [rad].
   * Derived from tilt sensing; polluted by centripetal acceleration during rotation.
   * Noisy but does not drift over time (unlike gyro integration).
   */
  accelAngleEstimate: number;
}

/**
 * Configuration parameters for the sensor model.
 */
export interface SensorParams {
  /**
   * Gyroscope noise standard deviation [rad/s].
   * PLACEHOLDER — representative of MPU6050 at 100 Hz (R1.4).
   */
  gyroNoiseSigma: number;

  /**
   * Accelerometer-derived angle noise standard deviation [rad].
   * PLACEHOLDER — representative of MPU6050 + tilt-conversion noise (R1.4).
   */
  accelAngleNoiseSigma: number;

  /**
   * Gyro bias drift rate [rad/s per second].
   * Models in-run bias instability (slow random walk of gyro zero-point).
   * PLACEHOLDER — representative of MPU6050 bias stability (R1.4).
   */
  gyroBiasDriftRate: number;

  /**
   * Sensor sample rate [Hz].
   * Must be ≤ physics rate. Sensor only updates once per sensor tick,
   * even if the physics engine has stepped multiple times since the last sample.
   */
  sampleRateHz: number;

  /**
   * PRNG seed for reproducible noise sequences.
   * Required by R2.4 (no Math.random) and R3.3 (deterministic tests).
   */
  seed: number;
}

/**
 * SensorModel interface — public contract for IMU simulation.
 * Designed so an alternative sensor model can be swapped in without
 * touching the simulation loop (PRD §6.5 extensibility hook).
 */
export interface SensorModel {
  /**
   * Attempts to produce a new sample for the given true physics state and elapsed time.
   * The sensor only updates internally at its configured sampleRateHz; between ticks
   * it returns the last valid reading (zero-order hold, matching real IMU behavior).
   *
   * @param trueState  Ground-truth rigid-body state from the physics engine.
   * @param dt         Elapsed physics timestep [s] since last call.
   * @returns          The current sensor reading (may be the same as last tick if
   *                   the sensor hasn't fired yet this physics step).
   */
  sample(trueState: RigidBodyState, dt: number): SensorReading;

  /** Resets sensor state (bias accumulation, internal clock) for a clean simulation reset. */
  reset(): void;

  /** Returns the current accumulated gyro bias [rad/s]. Exposed for telemetry/debug only. */
  getCurrentBias(): number;
}

/**
 * Default sensor model implementation — simulated MPU6050 equivalent.
 * Uses a seeded PRNG (Xorshift32 or equivalent) for all noise (R2.4).
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 2.
 */
export class DefaultSensorModel implements SensorModel {
  private readonly params: SensorParams;

  constructor(params: SensorParams) {
    this.params = params;
    void this.params;
  }

  sample(_trueState: RigidBodyState, _dt: number): SensorReading {
    throw new Error('not implemented');
  }

  reset(): void {
    throw new Error('not implemented');
  }

  getCurrentBias(): number {
    throw new Error('not implemented');
  }
}
