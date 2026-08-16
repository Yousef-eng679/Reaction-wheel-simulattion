/**
 * control.saturation.test.ts
 *
 * Actuator saturation stability test — system must not diverge under saturation.
 *
 * Tests (PRD §6.7, Phase 3 deliverable):
 *   Command an unreasonably large step target (10 full rotations) so the actuator
 *   saturates immediately and stays saturated for the entire 60-second run.
 *
 *   Test 1 — System does not diverge: state values stay bounded throughout.
 *   Test 2 — Body angle and angular velocity remain within explicit numeric bounds.
 *
 * Why these bounds?
 *   With maxTorqueNm = 0.05 N·m and I2 = 5e-3 kg·m², the maximum achievable
 *   angular acceleration of the body is 0.05/5e-3 = 10 rad/s².
 *   Over 60 s, a fully unconstrained integral of that would reach 36 000 rad.
 *   The fact that the wheel saturates its RPM limits the body's angular velocity to
 *   the momentum-exchange limit. We assert much tighter bounds than the theoretical
 *   worst case to catch any runaway divergence.
 *
 * Motor between-tick protocol:
 *   Same held duty-cycle pattern as control.stepResponse.test.ts — see that file.
 *
 * R3.2 — real physics engine used (not mocked).
 * R3.3 — fixed sensor seed.
 * R2.1 — pid.update() never receives trueState.
 */

import { describe, it, expect } from 'vitest';
import { integrate } from '../core/physics/physicsEngine.ts';
import { createZeroState } from '../core/physics/rigidBodyState.ts';
import { DefaultMotorModel } from '../core/actuator/motorModel.ts';
import { DefaultSensorModel } from '../core/sensor/sensorModel.ts';
import { ComplementaryFilter } from '../core/sensor/estimator.ts';
import { PidController } from '../core/control/pidController.ts';
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

/** Unreasonably large step — 10 full rotations, guaranteed to saturate the actuator. */
const LARGE_STEP_RAD = 10 * 2 * Math.PI; // ≈ 62.83 rad

// ---------------------------------------------------------------------------
// Shared closed-loop runner for saturation scenario
// ---------------------------------------------------------------------------

interface SatResult {
  maxAbsTheta2: number;
  maxAbsOmega2: number;
  finalIntegralNm: number;
}

function runSaturationScenario(durationS: number, seed: number): SatResult {
  const physicsParams = {
    I1: DEFAULT_I1_WHEEL_KGM2,
    I2: DEFAULT_I2_BODY_KGM2,
    frictionCoeff1: DEFAULT_FRICTION_COEFF_1,
    frictionCoeff2: DEFAULT_FRICTION_COEFF_2,
  };

  const motor = new DefaultMotorModel({
    maxRPM: DEFAULT_MOTOR_MAX_RPM,
    maxTorqueNm: DEFAULT_MOTOR_MAX_TORQUE_NM,
    timeConstantS: DEFAULT_MOTOR_TIME_CONSTANT_S,
  }, physicsParams);

  const sensor = new DefaultSensorModel({
    gyroNoiseSigma: DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
    accelAngleNoiseSigma: DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
    gyroBiasDriftRate: DEFAULT_GYRO_BIAS_DRIFT_RATE,
    sampleRateHz: DEFAULT_SENSOR_SAMPLE_RATE_HZ,
    seed,
  });

  const estimator = new ComplementaryFilter({ alpha: DEFAULT_COMPLEMENTARY_ALPHA });

  const pid = new PidController({
    kp: DEFAULT_KP,
    ki: DEFAULT_KI,
    kd: DEFAULT_KD,
    outputMin: -OUTPUT_MAX,
    outputMax: OUTPUT_MAX,
  });

  let state = createZeroState();

  const dt = PHYSICS_DT_S;
  const totalSteps = Math.round(durationS / dt);
  const sensorDt = 1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ;
  const stepsPerSensorTick = Math.round(sensorDt / dt);

  // Held duty cycle between control ticks (correct protocol — see stepResponse for rationale)
  let heldDutyCycle = 0;

  let maxAbsTheta2 = 0;
  let maxAbsOmega2 = 0;

  for (let i = 0; i < totalSteps; i++) {
    const reading = sensor.sample(state, dt);

    if (i % stepsPerSensorTick === 0) {
      estimator.update(reading, sensorDt);
      // R2.1: estimator output only, never trueState
      const pidOutputNm = pid.update(LARGE_STEP_RAD, estimator.getEstimate(), sensorDt);
      heldDutyCycle = -pidOutputNm / OUTPUT_MAX;
    }

    const actualTorqueNm = motor.update(heldDutyCycle, dt);
    state = integrate(state, actualTorqueNm, 0, dt, physicsParams);

    if (Math.abs(state.theta2) > maxAbsTheta2) maxAbsTheta2 = Math.abs(state.theta2);
    if (Math.abs(state.omega2) > maxAbsOmega2) maxAbsOmega2 = Math.abs(state.omega2);
  }

  return {
    maxAbsTheta2,
    maxAbsOmega2,
    finalIntegralNm: pid.getIntegralTerm(),
  };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('Control — actuator saturation', () => {
  /**
   * Test 1: The state must not diverge.
   *
   * Body angle bound rationale: The wheel can spin at most maxRPM and I1 is
   * known. The maximum angular momentum the wheel can hold is I1 × maxOmega.
   * By conservation, the body picks up at most that much in the opposite direction.
   * maxOmega ≈ 5400 × 2π/60 ≈ 565 rad/s; I1 = 1e-4; I2 = 5e-3
   * Max body omega from momentum exchange = I1/I2 × maxOmega ≈ 11.3 rad/s.
   * Over 60 s this integrates to ≈ 678 rad — we allow up to 800 rad.
   * (The saturation test is about no-divergence, not reaching the setpoint.)
   *
   * Angular velocity bound: body omega from momentum exchange ≤ I1/I2 × maxOmega ≈ 11.3 rad/s.
   * Allow 25 rad/s with margin.
   */
  it('body angle stays within physical momentum-exchange bound (state does not diverge)', () => {
    const { maxAbsTheta2, maxAbsOmega2 } = runSaturationScenario(60, /* seed */ 99);

    // Angular velocity must be bounded by the momentum-exchange limit
    // max body omega ≈ (I1/I2) × maxWheelOmega = (1e-4/5e-3) × 565 ≈ 11.3 rad/s
    // Allow 25 rad/s to accommodate transient effects
    expect(maxAbsOmega2, `Body angular velocity ${maxAbsOmega2.toFixed(3)} rad/s exceeded bound`)
      .toBeLessThan(25);

    // Angle should not exceed ≈800 rad (≈127 full rotations) — far below divergence
    expect(maxAbsTheta2, `Body angle ${maxAbsTheta2.toFixed(2)} rad exceeded bound`)
      .toBeLessThan(800);
  });

  /**
   * Test 2: Explicit numeric bounds on angle, angular velocity, and integral term.
   * Confirms no NaN/Infinity, and that windup protection kept the integral finite.
   */
  it('body angle and angular velocity remain finite; integral term bounded by windup protection', () => {
    const { maxAbsTheta2, maxAbsOmega2, finalIntegralNm } = runSaturationScenario(60, /* seed */ 99);

    // All values must be finite real numbers (guards against NaN/Infinity)
    expect(Number.isFinite(maxAbsTheta2), 'theta2 became NaN or Infinity').toBe(true);
    expect(Number.isFinite(maxAbsOmega2), 'omega2 became NaN or Infinity').toBe(true);
    expect(Number.isFinite(finalIntegralNm), 'integral became NaN or Infinity').toBe(true);

    // Integral wind-up guard: the conditional integration in PidController should
    // prevent the integral from growing beyond a bounded range even after 60 s of saturation.
    // Bound: if ki = 0.05 and error ≈ 62.83 rad and we allowed full integration for 60 s:
    //   max integral ≈ ki × error × t = 0.05 × 62.83 × 60 = 188.5 N·m·s (unguarded).
    // With windup protection active, it should stay well below this — assert < 10 N·m·s.
    expect(Math.abs(finalIntegralNm),
      `Integral ${finalIntegralNm.toFixed(4)} N·m·s — windup protection may not be working`)
      .toBeLessThan(10);
  });
});
