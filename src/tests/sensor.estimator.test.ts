/**
 * sensor.estimator.test.ts
 *
 * Unit tests for DefaultSensorModel and ComplementaryFilter (Phase 2).
 *
 * Two categories:
 *
 * 1. Sensor model: noise is Gaussian with the correct sigma, bias drifts
 *    slowly, and the PRNG is deterministic (same seed → same noise sequence).
 *
 * 2. Complementary filter: with realistic noise enabled, the filter estimate
 *    tracks a known ground-truth angle trajectory within a defined bounded
 *    error over time — proving it does NOT drift away like pure gyro integration.
 *
 * These tests do not use the physics engine (no integrate() calls). The true
 * state trajectory is driven analytically so the test is focused on sensor/
 * estimator behaviour only, not physics accuracy (which is Phase 1's job).
 *
 * R3.3 — All noise sequences are seeded; results are deterministic across runs.
 * R2.4 — No Math.random() in sensor code (verified by grepping sensor/).
 */

import { describe, it, expect } from 'vitest';
import { DefaultSensorModel } from '../core/sensor/sensorModel.ts';
import { ComplementaryFilter } from '../core/sensor/estimator.ts';
import type { SensorReading } from '../core/sensor/sensorModel.ts';
import {
  DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
  DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
  DEFAULT_GYRO_BIAS_DRIFT_RATE,
  DEFAULT_SENSOR_SAMPLE_RATE_HZ,
  DEFAULT_COMPLEMENTARY_ALPHA,
  PHYSICS_DT_S,
} from '../core/physics/constants.ts';

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

/**
 * Constructs a minimal true state for a given theta2 and omega2.
 * theta1/omega1 are zero — sensor model only reads theta2/omega2.
 */
function makeState(theta2: number, omega2: number) {
  return { theta1: 0, omega1: 0, theta2, omega2 };
}

const SENSOR_PARAMS = {
  gyroNoiseSigma: DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
  accelAngleNoiseSigma: DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
  gyroBiasDriftRate: DEFAULT_GYRO_BIAS_DRIFT_RATE,
  sampleRateHz: DEFAULT_SENSOR_SAMPLE_RATE_HZ,
  seed: 12345, // Fixed seed for determinism (R3.3)
};

// ---------------------------------------------------------------------------
// Sensor model tests
// ---------------------------------------------------------------------------

describe('Sensor — DefaultSensorModel', () => {
  /**
   * Determinism: same seed → identical reading sequence on two independent instances.
   */
  it('produces identical readings for the same seed (PRNG determinism)', () => {
    const sensorA = new DefaultSensorModel(SENSOR_PARAMS);
    const sensorB = new DefaultSensorModel(SENSOR_PARAMS);
    const state = makeState(0.3, 0.5);
    const dt = PHYSICS_DT_S;

    // Run both for 500 physics steps
    const readingsA: SensorReading[] = [];
    const readingsB: SensorReading[] = [];
    for (let i = 0; i < 500; i++) {
      readingsA.push(sensorA.sample(state, dt));
      readingsB.push(sensorB.sample(state, dt));
    }

    // Every reading must be identical
    for (let i = 0; i < readingsA.length; i++) {
      expect(readingsA[i].gyroOmega2).toBe(readingsB[i].gyroOmega2);
      expect(readingsA[i].accelAngleEstimate).toBe(readingsB[i].accelAngleEstimate);
    }
  });

  /**
   * Different seeds → different readings (PRNG is not constant).
   */
  it('produces different readings for different seeds', () => {
    const sensorA = new DefaultSensorModel({ ...SENSOR_PARAMS, seed: 1 });
    const sensorB = new DefaultSensorModel({ ...SENSOR_PARAMS, seed: 2 });
    const state = makeState(0, 0);
    const dt = PHYSICS_DT_S;

    // Advance to the first sensor tick (1/sampleRateHz = 5ms → 5 physics steps)
    for (let i = 0; i < 5; i++) {
      sensorA.sample(state, dt);
      sensorB.sample(state, dt);
    }
    const rA = sensorA.sample(state, dt);
    const rB = sensorB.sample(state, dt);

    // With different seeds, readings should differ at least somewhere
    const differ = rA.gyroOmega2 !== rB.gyroOmega2 ||
                   rA.accelAngleEstimate !== rB.accelAngleEstimate;
    expect(differ).toBe(true);
  });

  /**
   * reset() resets the PRNG and bias, so the reading sequence restarts identically.
   */
  it('reset() produces the same sequence as a fresh instance', () => {
    const sensor = new DefaultSensorModel(SENSOR_PARAMS);
    const state = makeState(0.1, 0.2);
    const dt = PHYSICS_DT_S;

    // Collect 300 readings, then reset and collect again
    const run1: SensorReading[] = [];
    for (let i = 0; i < 300; i++) run1.push(sensor.sample(state, dt));

    sensor.reset();

    const run2: SensorReading[] = [];
    for (let i = 0; i < 300; i++) run2.push(sensor.sample(state, dt));

    for (let i = 0; i < run1.length; i++) {
      expect(run1[i].gyroOmega2).toBe(run2[i].gyroOmega2);
      expect(run1[i].accelAngleEstimate).toBe(run2[i].accelAngleEstimate);
    }
  });

  /**
   * Gyro noise: with zero bias drift and a static state, the mean of gyro
   * readings should be close to the true omega2 and spread ≈ gyroNoiseSigma.
   * Uses enough samples (n=2000) that Central Limit Theorem applies.
   */
  it('gyro readings are statistically centred on trueOmega2 with correct sigma', () => {
    const sensor = new DefaultSensorModel({
      ...SENSOR_PARAMS,
      gyroBiasDriftRate: 0, // Zero drift so mean tracks true omega2 cleanly
    });
    const trueOmega2 = 1.5; // [rad/s]
    const state = makeState(0, trueOmega2);
    const dt = PHYSICS_DT_S;
    const stepsPerTick = Math.round((1 / SENSOR_PARAMS.sampleRateHz) / dt); // 5

    // Collect exactly n distinct sensor readings by sampling only at tick boundaries
    const n = 2000;
    const sensorReadings: number[] = [];
    for (let i = 0; i < n * stepsPerTick; i++) {
      const r = sensor.sample(state, dt);
      // Record the reading at the start of each new sensor tick (index 0 of each group)
      if (i % stepsPerTick === stepsPerTick - 1) {
        sensorReadings.push(r.gyroOmega2);
      }
    }

    const sample = sensorReadings.slice(0, n);
    const mean = sample.reduce((a, b) => a + b, 0) / sample.length;
    const variance = sample.reduce((s, v) => s + (v - mean) ** 2, 0) / sample.length;
    const stddev = Math.sqrt(variance);

    // Mean should be within 5σ/√n of trueOmega2 (generous to handle PRNG quirks)
    const meanTolerance = 5 * DEFAULT_GYRO_NOISE_SIGMA_RAD_S / Math.sqrt(sample.length);
    expect(Math.abs(mean - trueOmega2)).toBeLessThan(meanTolerance);

    // Measured stddev should be within 25% of configured gyroNoiseSigma
    expect(Math.abs(stddev - DEFAULT_GYRO_NOISE_SIGMA_RAD_S))
      .toBeLessThan(0.25 * DEFAULT_GYRO_NOISE_SIGMA_RAD_S);
  });

  /**
   * Zero-order hold: sensor should return the same reading for multiple physics
   * steps between sensor ticks (e.g., sampleRateHz=200 Hz → fires every 5ms,
   * so 5 consecutive calls at dt=1ms return the same value).
   */
  it('holds the last reading between sensor ticks (zero-order hold)', () => {
    const sensor = new DefaultSensorModel(SENSOR_PARAMS);
    const state = makeState(0, 0);
    const dt = PHYSICS_DT_S; // 1ms
    const sensorPeriod = 1 / SENSOR_PARAMS.sampleRateHz; // 5ms
    const stepsPerTick = Math.round(sensorPeriod / dt); // 5 steps

    // Advance through one complete tick so the sensor has a nonzero lastReading.
    // After stepsPerTick calls, the accumulator has just reset after firing one tick.
    for (let i = 0; i < stepsPerTick; i++) sensor.sample(state, dt);

    // The reading at the end of the last tick is now held.
    // The NEXT stepsPerTick-1 calls should return the exact same value
    // (accumulator hasn't reached sensorPeriod again yet).
    const held = sensor.sample(state, dt); // step 1 of new period — no tick yet
    for (let i = 2; i < stepsPerTick; i++) {
      const r = sensor.sample(state, dt);
      expect(r.gyroOmega2).toBe(held.gyroOmega2);
      expect(r.accelAngleEstimate).toBe(held.accelAngleEstimate);
    }
  });
});

// ---------------------------------------------------------------------------
// Complementary filter tests
// ---------------------------------------------------------------------------

describe('Sensor — ComplementaryFilter estimator', () => {
  /**
   * Core tracking test: with realistic noise enabled, the complementary filter
   * estimate must track a known sinusoidal ground-truth angle within a defined
   * bounded error. This proves the filter doesn't drift away from truth.
   *
   * Scenario:
   *   True trajectory: θ₂(t) = 0.4 × sin(2π × 0.05 × t)  [rad, 0.05 Hz slow swing]
   *   True ω₂(t) = 0.4 × 2π × 0.05 × cos(2π × 0.05 × t) [rad/s]
   *   Duration: 30 s (1.5 full oscillations)
   *
   * After an initial warm-up period (3 s), the estimate error must remain
   * within ±0.08 rad (≈ ±4.6°) at every logged step for the remainder of the run.
   * This is explicitly defined — not "looks OK."
   */
  it('tracks sinusoidal ground truth within ±0.08 rad after 3s warm-up (realistic noise)', () => {
    const sensor = new DefaultSensorModel(SENSOR_PARAMS);
    const filter = new ComplementaryFilter({ alpha: DEFAULT_COMPLEMENTARY_ALPHA });

    const dt = PHYSICS_DT_S; // 1ms physics steps
    const sensorDt = 1 / SENSOR_PARAMS.sampleRateHz; // 5ms sensor ticks

    const AMPLITUDE = 0.4;   // [rad]
    const FREQ_HZ   = 0.05;  // [Hz] — slow oscillation
    const DURATION_SEC   = 30;
    const WARMUP_SEC     = 3;
    const ERROR_BOUND    = 0.08; // [rad] ≈ 4.6° — explicitly defined bound

    const nSteps = Math.round(DURATION_SEC / dt);

    let maxErrorAfterWarmup = 0;

    for (let i = 0; i < nSteps; i++) {
      const t = i * dt;

      // Analytical ground truth
      const trueTheta2 = AMPLITUDE * Math.sin(2 * Math.PI * FREQ_HZ * t);
      const trueOmega2 = AMPLITUDE * 2 * Math.PI * FREQ_HZ * Math.cos(2 * Math.PI * FREQ_HZ * t);
      const state = makeState(trueTheta2, trueOmega2);

      // Sensor sample (fires every sensorDt)
      const reading = sensor.sample(state, dt);

      // Only update the filter at sensor ticks (every 5 physics steps)
      if (i % Math.round(sensorDt / dt) === 0) {
        filter.update(reading, sensorDt);
      }

      // Log error after warm-up
      if (t >= WARMUP_SEC) {
        const error = Math.abs(filter.getEstimate() - trueTheta2);
        if (error > maxErrorAfterWarmup) maxErrorAfterWarmup = error;
      }
    }

    // The maximum error after warm-up must be within the stated bound
    expect(maxErrorAfterWarmup).toBeLessThan(ERROR_BOUND);
  });

  /**
   * Anti-drift proof: pure gyro integration diverges due to bias, but the
   * complementary filter stays bounded.
   *
   * Scenario: body held stationary at θ₂ = 0.
   *   Gyro reading = 0 + noise + accumulated bias.
   *   Pure gyro integration: θ_pure += gyro × dt → drifts steadily.
   *   Complementary filter: accel term anchors it to 0, preventing drift.
   *
   * After 60 seconds:
   *   - Pure gyro integration error must be significantly larger than the filter error.
   *   - Filter error must stay within a bounded tolerance.
   */
  it('filter stays bounded while pure-gyro integration drifts (proof of anti-drift)', () => {
    const sensor = new DefaultSensorModel({
      ...SENSOR_PARAMS,
      gyroBiasDriftRate: 1e-3, // Amplified bias drift rate to make drift clearly visible
      seed: 9999,
    });
    const filter = new ComplementaryFilter({ alpha: DEFAULT_COMPLEMENTARY_ALPHA });

    const dt = PHYSICS_DT_S;
    const sensorDt = 1 / SENSOR_PARAMS.sampleRateHz;
    const DURATION_SEC = 60;
    const nSteps = Math.round(DURATION_SEC / dt);

    // True state: body at rest, angle = 0
    const state = makeState(0, 0);

    let pureGyroIntegral = 0; // gyro-only angle estimate [rad]

    for (let i = 0; i < nSteps; i++) {
      const reading = sensor.sample(state, dt);

      if (i % Math.round(sensorDt / dt) === 0) {
        // Update complementary filter
        filter.update(reading, sensorDt);
        // Update pure gyro integration (no accel correction)
        pureGyroIntegral += reading.gyroOmega2 * sensorDt;
      }
    }

    const filterError = Math.abs(filter.getEstimate());  // true angle = 0
    const pureGyroError = Math.abs(pureGyroIntegral);

    // Pure gyro must have drifted noticeably more than the filter
    expect(pureGyroError).toBeGreaterThan(filterError * 5);

    // Filter must stay within a reasonable bound (< 0.15 rad = ~8.6°)
    expect(filterError).toBeLessThan(0.15);
  });

  /**
   * reset() clears the estimate — after reset, filter starts from zero.
   */
  it('reset() returns estimate to zero', () => {
    const filter = new ComplementaryFilter({ alpha: DEFAULT_COMPLEMENTARY_ALPHA });
    const sensor = new DefaultSensorModel(SENSOR_PARAMS);
    const state = makeState(1.0, 0.5);
    const sensorDt = 1 / SENSOR_PARAMS.sampleRateHz;

    // Run for a while to build up a nonzero estimate
    for (let i = 0; i < 200; i++) {
      const reading = sensor.sample(state, PHYSICS_DT_S);
      filter.update(reading, sensorDt);
    }
    expect(Math.abs(filter.getEstimate())).toBeGreaterThan(0.1);

    filter.reset();
    expect(filter.getEstimate()).toBe(0);
  });

  /**
   * With zero noise and zero bias, the filter should converge exactly to
   * the true angle (steady-state: accel term pulls to truth, gyro confirms).
   */
  it('converges to true angle with zero noise and zero bias', () => {
    const sensor = new DefaultSensorModel({
      gyroNoiseSigma: 0,
      accelAngleNoiseSigma: 0,
      gyroBiasDriftRate: 0,
      sampleRateHz: DEFAULT_SENSOR_SAMPLE_RATE_HZ,
      seed: 1,
    });
    const filter = new ComplementaryFilter({ alpha: DEFAULT_COMPLEMENTARY_ALPHA });

    const TARGET_ANGLE = 0.5; // [rad]
    const state = makeState(TARGET_ANGLE, 0); // body stationary at 0.5 rad
    const dt = PHYSICS_DT_S;
    const sensorDt = 1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ;

    for (let i = 0; i < Math.round(10 / dt); i++) { // 10 seconds
      const reading = sensor.sample(state, dt);
      if (i % Math.round(sensorDt / dt) === 0) {
        filter.update(reading, sensorDt);
      }
    }

    // Should be within 0.001 rad of true angle
    expect(Math.abs(filter.getEstimate() - TARGET_ANGLE)).toBeLessThan(0.001);
  });
});
