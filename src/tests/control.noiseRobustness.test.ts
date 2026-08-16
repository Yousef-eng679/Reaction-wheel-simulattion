/**
 * control.noiseRobustness.test.ts
 *
 * Sensor noise robustness test — controller must converge despite realistic noise.
 *
 * Tests (PRD §6.7, Phase 6 deliverable):
 *   With realistic sensor noise levels enabled (default sigma values), run the
 *   closed-loop system for a step-response scenario and assert:
 *     - The estimated angle converges to within a wider (but explicitly defined)
 *       tolerance band around the target — not perfect precision, but bounded error.
 *     - The system does NOT oscillate divergently.
 *
 * The tolerance band here is wider than the clean-physics test (Phase 3), since
 * sensor noise adds a realistic floor to achievable precision.
 *
 * Phase 0: test structure scaffold only.
 *   - Imports are correct.
 *   - describe/it structure is in place.
 *   - Test bodies verify the stub throws 'not implemented'.
 *
 * Phase 6 will implement the full noise robustness assertions using scenarioRunner.
 */

import { describe, it, expect } from 'vitest';
import { runScenario } from '../sim/scenarioRunner.ts';
import type { ScenarioConfig } from '../sim/scenarioRunner.ts';
import {
  DEFAULT_I1_WHEEL_KGM2,
  DEFAULT_I2_BODY_KGM2,
  DEFAULT_FRICTION_COEFF_1,
  DEFAULT_FRICTION_COEFF_2,
  DEFAULT_MOTOR_MAX_RPM,
  DEFAULT_MOTOR_MAX_TORQUE_NM,
  DEFAULT_MOTOR_TIME_CONSTANT_S,
  DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
  DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
  DEFAULT_GYRO_BIAS_DRIFT_RATE,
  DEFAULT_SENSOR_SAMPLE_RATE_HZ,
  DEFAULT_COMPLEMENTARY_ALPHA,
  PHYSICS_DT_S,
  DEFAULT_KP,
  DEFAULT_KI,
  DEFAULT_KD,
  DEFAULT_MOTOR_MAX_TORQUE_NM as OUTPUT_MAX,
} from '../core/physics/constants.ts';

/** Target: 30 degrees in radians. */
const STEP_TARGET_RAD = Math.PI / 6; // 30°

/**
 * Tolerance band for noisy convergence: ±5 degrees (wider than the clean ±2° bound).
 * Explicitly defined — not just "close enough."
 */
const NOISY_CONVERGENCE_TOLERANCE_RAD = 5 * (Math.PI / 180); // 5° in rad

const NOISE_ROBUSTNESS_SCENARIO: ScenarioConfig = {
  name: 'Noise robustness — realistic sensor noise, 30° step',
  simConfig: {
    physicsParams: {
      I1: DEFAULT_I1_WHEEL_KGM2,
      I2: DEFAULT_I2_BODY_KGM2,
      frictionCoeff1: DEFAULT_FRICTION_COEFF_1,
      frictionCoeff2: DEFAULT_FRICTION_COEFF_2,
    },
    motorParams: {
      maxRPM: DEFAULT_MOTOR_MAX_RPM,
      maxTorqueNm: DEFAULT_MOTOR_MAX_TORQUE_NM,
      timeConstantS: DEFAULT_MOTOR_TIME_CONSTANT_S,
    },
    sensorParams: {
      // Full realistic noise — not zeroed out like Phase 1 conservation test.
      gyroNoiseSigma: DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
      accelAngleNoiseSigma: DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
      gyroBiasDriftRate: DEFAULT_GYRO_BIAS_DRIFT_RATE,
      sampleRateHz: DEFAULT_SENSOR_SAMPLE_RATE_HZ,
      seed: 7, // Fixed seed (R3.3)
    },
    estimatorParams: {
      alpha: DEFAULT_COMPLEMENTARY_ALPHA,
    },
    pidParams: {
      kp: DEFAULT_KP,
      ki: DEFAULT_KI,
      kd: DEFAULT_KD,
      outputMin: -OUTPUT_MAX,
      outputMax: OUTPUT_MAX,
    },
  },
  setpointRad: STEP_TARGET_RAD,
  durationSec: 60,
  loggingIntervalS: PHYSICS_DT_S * 10,
};

// Export for use in Phase 6 test assertions.
export { NOISY_CONVERGENCE_TOLERANCE_RAD };

describe('Control — sensor noise robustness', () => {
  it('converges to within 5° of target despite realistic sensor noise', () => {
    // Phase 0 scaffold: throws 'not implemented' until Phase 6.
    expect(() => {
      runScenario(NOISE_ROBUSTNESS_SCENARIO);
    }).toThrow('not implemented');
  });

  it('does not exhibit divergent oscillation under realistic noise', () => {
    expect(() => {
      runScenario(NOISE_ROBUSTNESS_SCENARIO);
    }).toThrow('not implemented');
  });
});
