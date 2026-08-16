/**
 * control.stepResponse.test.ts
 *
 * PID step response test — closed-loop behavior with real physics engine.
 *
 * Tests (PRD §6.7, Phase 3 deliverable):
 *   Using real physics engine (NOT mocked — R3.2), command a 45-degree step
 *   target angle. Assert that with a known-good default gain set:
 *     - The body settles within ±2 degrees of target within a defined settle time.
 *     - Overshoot does not exceed a defined percentage limit.
 *
 * Phase 0: test structure scaffold only.
 *   - Imports are correct (using runScenario from scenarioRunner).
 *   - describe/it structure is in place.
 *   - Test bodies verify that the stub throws 'not implemented'.
 *
 * Phase 3 will implement the full closed-loop assertions (real physics, no mocks).
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

/** Target angle for step response test: 45 degrees in radians. */
const STEP_TARGET_RAD = Math.PI / 4; // 45°

const STEP_SCENARIO: ScenarioConfig = {
  name: 'Step response — 45° step, default gains',
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
      gyroNoiseSigma: DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
      accelAngleNoiseSigma: DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
      gyroBiasDriftRate: DEFAULT_GYRO_BIAS_DRIFT_RATE,
      sampleRateHz: DEFAULT_SENSOR_SAMPLE_RATE_HZ,
      seed: 42, // Fixed seed for determinism (R3.3)
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
  durationSec: 30,
  loggingIntervalS: PHYSICS_DT_S * 10,
};

describe('Control — PID step response', () => {
  it('settles within ±2° of 45° target within 30s with default gains', () => {
    // Phase 0 scaffold: runScenario throws 'not implemented' until Phase 4.
    // Phase 3 will add the real settling and overshoot assertions.
    expect(() => {
      runScenario(STEP_SCENARIO);
    }).toThrow('not implemented');
  });

  it('overshoot does not exceed 20% of step magnitude', () => {
    expect(() => {
      runScenario(STEP_SCENARIO);
    }).toThrow('not implemented');
  });
});
