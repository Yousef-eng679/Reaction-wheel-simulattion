/**
 * sensorModel.ts
 *
 * Sensor model: simulated MPU6050-equivalent IMU (PRD §6.3).
 *
 * Simulates what a real MPU6050 delivers:
 *   - Gyroscope: body angular velocity ω₂ + Gaussian noise + slow bias drift
 *   - Accelerometer: tilt-derived angle θ₂ + Gaussian noise + centripetal pollution
 *
 * Key implementation decisions:
 *   - Seedable PRNG (Xorshift32) for ALL randomness. Math.random() is FORBIDDEN
 *     here (R2.4) — it would break deterministic, reproducible tests (R3.3).
 *   - Gaussian noise via Box-Muller transform applied to PRNG output.
 *   - Zero-order hold: sensor only updates at sampleRateHz; between sensor ticks
 *     it returns the last valid reading, matching real IMU behaviour.
 *   - Gyro bias drift: slow random walk modelled as per-sample Gaussian increment
 *     scaled by drift rate and sensor dt.
 *
 * R2.4 — Math.random() does NOT appear anywhere in this file.
 * R1.3 — All outputs in SI units (rad, rad/s). No conversion to degrees here.
 * R1.4 — All noise/timing parameters are named fields, no magic literals.
 */

import type { RigidBodyState } from '../physics/rigidBodyState.ts';

// ---------------------------------------------------------------------------
// Re-exported types (defined here, imported by estimator.ts and tests)
// ---------------------------------------------------------------------------

export interface SensorReading {
  /** Gyro angular velocity of body [rad/s]: true ω₂ + noise + bias drift. */
  gyroOmega2: number;
  /**
   * Accelerometer-derived body angle estimate [rad].
   * Noisy but drift-free; polluted by centripetal acceleration during fast rotation.
   */
  accelAngleEstimate: number;
}

export interface SensorParams {
  /** Gyro noise standard deviation [rad/s]. PLACEHOLDER (R1.4). */
  gyroNoiseSigma: number;
  /** Accel-derived angle noise standard deviation [rad]. PLACEHOLDER (R1.4). */
  accelAngleNoiseSigma: number;
  /**
   * Gyro bias drift rate [rad/s per second].
   * Controls in-run bias instability (slow random walk of the gyro's zero point).
   * PLACEHOLDER (R1.4).
   */
  gyroBiasDriftRate: number;
  /** Sensor sample rate [Hz]. Must be ≤ physics rate. */
  sampleRateHz: number;
  /** PRNG seed for deterministic noise (R2.4, R3.3). */
  seed: number;
}

export interface SensorModel {
  sample(trueState: RigidBodyState, dt: number): SensorReading;
  reset(): void;
  getCurrentBias(): number;
}

// ---------------------------------------------------------------------------
// Internal: Xorshift32 seedable PRNG (R2.4 — replaces Math.random() entirely)
// ---------------------------------------------------------------------------

/**
 * Xorshift32 pseudo-random number generator.
 * Period: 2³² − 1. State is a single 32-bit unsigned integer.
 * Returns values in [0, 1) — same contract as Math.random(), but seedable.
 *
 * R2.4: This is the only source of randomness in core/sensor/.
 *       Math.random() must not appear anywhere in this directory.
 */
class Xorshift32 {
  private state: number;

  constructor(seed: number) {
    // Xorshift is undefined for state=0; clamp to 1.
    this.state = (seed >>> 0) || 1;
  }

  /** Returns next float in [0, 1). */
  nextFloat(): number {
    let x = this.state;
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    this.state = x >>> 0;
    return this.state / 0x100000000;
  }
}

// ---------------------------------------------------------------------------
// Internal: Box-Muller Gaussian sampler
// ---------------------------------------------------------------------------

/**
 * Returns one Gaussian-distributed sample with given mean and sigma,
 * using the Box-Muller transform applied to two PRNG draws.
 *
 * Box-Muller: if U₁, U₂ ~ Uniform(0,1), then
 *   Z = √(−2 ln U₁) · cos(2π U₂)  ~  N(0, 1)
 *
 * We guard against U₁ = 0 (ln(0) = −∞) with a small epsilon clamp.
 */
function sampleGaussian(prng: Xorshift32, mean: number, sigma: number): number {
  const u1 = Math.max(prng.nextFloat(), 1e-15); // guard ln(0)
  const u2 = prng.nextFloat();
  const z = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  return mean + sigma * z;
}

// ---------------------------------------------------------------------------
// DefaultSensorModel implementation
// ---------------------------------------------------------------------------

/**
 * DefaultSensorModel — simulated MPU6050-equivalent IMU.
 *
 * Internal timing: accumulates physics dt until ≥ 1/sampleRateHz, then fires
 * one sensor sample and updates lastReading. Between sensor ticks, sample()
 * returns the stale reading unchanged (zero-order hold — matches real IMU behaviour).
 *
 * Gyro bias drift: each sensor tick, the bias is nudged by a Gaussian increment
 * with sigma = driftRate × sensorDt. This simulates in-run bias instability:
 * the bias wanders slowly over time, causing gyro-only integration to drift.
 * The complementary filter's accelerometer term corrects this drift.
 *
 * Centripetal pollution of the accelerometer: during fast body rotation, the
 * accelerometer is biased by centripetal acceleration. This is modelled as an
 * additive term proportional to ω₂² × mounting_radius. For a small demo rig
 * this effect is small, so mounting_radius defaults to a representative 0.05 m.
 * The term is included so it can be increased in tests to verify filter robustness.
 *
 * R2.4 — All randomness goes through Xorshift32. No Math.random() calls here.
 */
export class DefaultSensorModel implements SensorModel {
  private readonly params: SensorParams;

  /** Pre-computed sensor tick period [s] = 1 / sampleRateHz. */
  private readonly sensorDt: number;

  /** Xorshift32 PRNG, seeded from params.seed. */
  private prng: Xorshift32;

  /** Accumulated physics time since last sensor tick [s]. */
  private timeAccumulator: number = 0;

  /** Current gyro bias [rad/s] — evolves via slow random walk. */
  private currentBias: number = 0;

  /** Last sensor reading, returned between sensor ticks (zero-order hold). */
  private lastReading: SensorReading = { gyroOmega2: 0, accelAngleEstimate: 0 };

  /**
   * Approximate body radius for centripetal acceleration modelling [m].
   * Centripetal pollution ≈ ω₂² × mountingRadius, projected onto tilt axis.
   * PLACEHOLDER — 0.05 m is representative of a small demo platform (R1.4).
   */
  private static readonly MOUNTING_RADIUS_M: number = 0.05; // [m] PLACEHOLDER

  constructor(params: SensorParams) {
    this.params = params;
    this.sensorDt = 1 / params.sampleRateHz;
    // Fresh PRNG seeded from params — same seed = identical noise sequence (R3.3).
    this.prng = new Xorshift32(params.seed);
  }

  /**
   * Advances the sensor clock by dt [s]. If the sensor has accumulated enough
   * time for a new tick, produces a fresh reading; otherwise returns the last one.
   *
   * @param trueState  Ground-truth physics state (ω₂, θ₂ are used).
   * @param dt         Elapsed physics timestep [s].
   * @returns          Current sensor reading (fresh or zero-order held).
   */
  sample(trueState: RigidBodyState, dt: number): SensorReading {
    this.timeAccumulator += dt;

    // Fire one or more sensor ticks if enough time has accumulated.
    // Normally exactly one tick fires per several physics steps, but we loop
    // to handle cases where dt > sensorDt (shouldn't happen at 1kHz physics,
    // but be defensive).
    while (this.timeAccumulator >= this.sensorDt) {
      this.timeAccumulator -= this.sensorDt;
      this.lastReading = this.computeReading(trueState);
    }

    return this.lastReading;
  }

  /** Resets sensor state (bias, clock, PRNG) for a clean simulation restart. */
  reset(): void {
    this.timeAccumulator = 0;
    this.currentBias = 0;
    // Re-seed to the same initial seed so a reset produces identical noise (R3.3).
    this.prng = new Xorshift32(this.params.seed);
    this.lastReading = { gyroOmega2: 0, accelAngleEstimate: 0 };
  }

  /** Returns current accumulated gyro bias [rad/s]. Exposed for telemetry only. */
  getCurrentBias(): number {
    return this.currentBias;
  }

  // ---------------------------------------------------------------------------
  // Private helpers
  // ---------------------------------------------------------------------------

  /**
   * Produces one sensor reading from the current true physics state.
   * Called once per sensor tick.
   */
  private computeReading(trueState: RigidBodyState): SensorReading {
    // --- Gyro bias drift (random walk, one step per sensor tick) ---
    // Bias sigma per tick = driftRate × sqrt(sensorDt) [rad/s]
    // (Random walk: variance grows linearly with time → sigma grows as sqrt(t))
    const biasSigma = this.params.gyroBiasDriftRate * Math.sqrt(this.sensorDt);
    this.currentBias += sampleGaussian(this.prng, 0, biasSigma);

    // --- Gyro reading: true ω₂ + white noise + accumulated bias ---
    const gyroNoise = sampleGaussian(this.prng, 0, this.params.gyroNoiseSigma);
    const gyroOmega2 = trueState.omega2 + gyroNoise + this.currentBias;

    // --- Accelerometer-derived angle: true θ₂ + white noise + centripetal pollution ---
    // Centripetal pollution: during body rotation, radial acceleration projects
    // onto the tilt axis, biasing the gravity-tilt estimate.
    // Magnitude ≈ ω₂² × R / g  [rad], where R = mounting radius and g = 9.81 m/s².
    const G_MS2 = 9.81; // [m/s²] — only used for centripetal bias, not SI-critical
    const centripetalBias =
      (trueState.omega2 * trueState.omega2 * DefaultSensorModel.MOUNTING_RADIUS_M) / G_MS2;
    const accelNoise = sampleGaussian(this.prng, 0, this.params.accelAngleNoiseSigma);
    const accelAngleEstimate = trueState.theta2 + accelNoise + centripetalBias;

    return { gyroOmega2, accelAngleEstimate };
  }
}
