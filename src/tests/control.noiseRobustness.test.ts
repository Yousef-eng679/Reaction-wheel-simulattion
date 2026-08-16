/**
 * control.noiseRobustness.test.ts
 *
 * Sensor noise robustness test — controller must converge despite realistic noise.
 *
 * Tests (PRD §6.7, Phase 4 upgrade — was Phase 6 scaffold):
 *   With realistic sensor noise levels enabled (default sigma values), run the
 *   closed-loop system for a 30° step-response scenario via runScenario() and assert:
 *
 *   Test 1 — Convergence: the TRUE body angle (not the estimate) must be within
 *     ±5° of the target during the final 10 s of a 60 s run. The wider tolerance
 *     (vs Phase 3's ±2°) accounts for the noise floor and sensor lag.
 *
 *   Test 2 — No divergent oscillation: the body angular velocity (ω₂) must remain
 *     bounded during the final 10 s. If the controller is oscillating divergently
 *     under noise, ω₂ will grow without bound; we assert it stays small.
 *
 * R3.2 — exercises real physics engine via SimulationLoop (inside runScenario).
 * R3.3 — fixed sensor seed.
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
  /**
   * Test 1: Convergence with real noise.
   * Evaluate the final 10 s of a 60 s run. Every snapshot in the last 10 s must
   * have trueState.theta2 within ±5° of the target. This proves the system has
   * genuinely settled (not just briefly crossed the target and drifted away).
   */
  it('true body angle stays within ±5° of target during final 10 s despite realistic sensor noise', () => {
    const result = runScenario(NOISE_ROBUSTNESS_SCENARIO);
    const { telemetry } = result;

    // Examine only the final 10 s of the 60 s run
    const evaluationWindowS = 10;
    const finalWindowSnaps = telemetry.filter(
      snap => snap.simTimeSec >= (NOISE_ROBUSTNESS_SCENARIO.durationSec - evaluationWindowS),
    );

    expect(finalWindowSnaps.length, 'No snapshots in final evaluation window').toBeGreaterThan(0);

    for (const snap of finalWindowSnaps) {
      expect(
        Math.abs(snap.trueState.theta2 - NOISE_ROBUSTNESS_SCENARIO.setpointRad),
        `True angle out of band at t=${snap.simTimeSec.toFixed(2)}s: ` +
        `theta2=${(snap.trueState.theta2 * 180 / Math.PI).toFixed(2)}°`,
      ).toBeLessThanOrEqual(NOISY_CONVERGENCE_TOLERANCE_RAD);
    }
  });

  /**
   * Test 2: No divergent oscillation.
   * Compute the RMS body angular velocity ω₂ over the final 10 s. If the controller
   * is oscillating divergently under noise, ω₂ will be large. A settled system should
   * have very small ω₂ — we bound it to < 0.1 rad/s.
   */
  it('body angular velocity stays small (< 0.1 rad/s RMS) in final 10 s — no divergent oscillation', () => {
    const result = runScenario(NOISE_ROBUSTNESS_SCENARIO);
    const { telemetry } = result;

    const evaluationWindowS = 10;
    const finalWindowSnaps = telemetry.filter(
      snap => snap.simTimeSec >= (NOISE_ROBUSTNESS_SCENARIO.durationSec - evaluationWindowS),
    );

    expect(finalWindowSnaps.length).toBeGreaterThan(0);

    const sumSqOmega2 = finalWindowSnaps.reduce(
      (acc, snap) => acc + snap.trueState.omega2 ** 2,
      0,
    );
    const rmsOmega2 = Math.sqrt(sumSqOmega2 / finalWindowSnaps.length);

    expect(
      rmsOmega2,
      `RMS ω₂ = ${rmsOmega2.toFixed(4)} rad/s — system may be diverging under noise`,
    ).toBeLessThan(0.1);
  });
});
