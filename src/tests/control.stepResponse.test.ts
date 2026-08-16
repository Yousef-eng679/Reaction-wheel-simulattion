/**
 * control.stepResponse.test.ts
 *
 * PID step response test — closed-loop behavior with real physics engine.
 *
 * Tests (PRD §6.7, Phase 3 deliverable):
 *   Using real physics engine (NOT mocked — R3.2), command a 45-degree step
 *   target angle. Assert that with a known-good default gain set:
 *     Test 1 — The body settles within ±2° of target within 30 s.
 *     Test 2 — Overshoot does not exceed 20% of the step magnitude.
 *
 * Closed-loop wiring (matching Phase 4 SimulationLoop architecture):
 *   physics → sensor → estimator → PID → motor (duty cycle) → torque → physics
 *
 * Motor between-tick protocol:
 *   The PID controller and estimator run at the sensor rate (200 Hz).
 *   Between sensor ticks the motor continues to receive the SAME duty cycle
 *   that was last commanded — not a re-derived value from the returned torque.
 *   This correctly models a real embedded system where the control interrupt fires
 *   at the sensor rate and holds the PWM duty between interrupts.
 *
 * R3.2 — real physics engine used (integrate() called directly, not mocked).
 * R3.3 — fixed sensor seed for deterministic results.
 * R2.1 — pid.update() receives estimator output only, never trueState.
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

/** Target angle: 45° in radians [rad]. */
const STEP_TARGET_RAD = Math.PI / 4;

/** Settle band: ±2° in radians [rad]. */
const TOLERANCE_RAD = (2 * Math.PI) / 180;

/** Settle duration: body must stay inside the tolerance band for this long [s]. */
const SETTLE_HOLD_SEC = 2.0;

/** Maximum allowed overshoot fraction (20% of step magnitude). */
const MAX_OVERSHOOT_FRACTION = 0.20;

// ---------------------------------------------------------------------------
// Shared closed-loop runner — runs the full loop for durationS seconds,
// returns { settledTimeS, maxOvershootRad, finalTheta2 }.
// Extracted so both it() blocks share identical physics wiring.
// ---------------------------------------------------------------------------

interface RunResult {
  /** First time [s] the body entered and held ±2° of target for ≥ SETTLE_HOLD_SEC. −1 if never. */
  settledTimeS: number;
  /** Maximum overshoot above the setpoint [rad]. 0 if body never exceeded setpoint. */
  maxOvershootRad: number;
  /** Body angle at end of run [rad]. */
  finalTheta2: number;
}

function runStepScenario(durationS: number, seed: number): RunResult {
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

  const dt = PHYSICS_DT_S;                                        // 1 ms
  const totalSteps = Math.round(durationS / dt);
  const sensorDt = 1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ;            // 5 ms
  const stepsPerSensorTick = Math.round(sensorDt / dt);          // 5

  // Duty cycle held between control ticks (correct protocol: hold PWM, not re-derive torque)
  let heldDutyCycle = 0;

  let maxOvershootRad = 0;
  let settledTimeS = -1;
  let continuousSettledSteps = 0;
  const settleHoldSteps = Math.round(SETTLE_HOLD_SEC / dt);

  for (let i = 0; i < totalSteps; i++) {
    const t = i * dt;

    // --- Sensor sample (every physics step; sensor model uses zero-order hold internally) ---
    const reading = sensor.sample(state, dt);

    // --- Control tick (at sensor rate only) ---
    if (i % stepsPerSensorTick === 0) {
      // Estimator update — only reads sensor, never trueState (R2.1)
      estimator.update(reading, sensorDt);

      // PID update — receives estimator output only, never trueState (R2.1)
      // Positive error (body below target) → positive PID output → negative duty cycle
      // (spinning wheel negatively applies positive body torque; see physics sign convention)
      const pidOutputNm = pid.update(STEP_TARGET_RAD, estimator.getEstimate(), sensorDt);
      heldDutyCycle = -pidOutputNm / OUTPUT_MAX;   // [−1, 1]
    }

    // --- Motor update — always called with the held duty cycle, every physics step ---
    const actualTorqueNm = motor.update(heldDutyCycle, dt);

    // --- Physics integration (RK4) ---
    state = integrate(state, actualTorqueNm, 0, dt, physicsParams);

    // --- Metrics ---
    const overshoot = state.theta2 - STEP_TARGET_RAD;
    if (overshoot > maxOvershootRad) maxOvershootRad = overshoot;

    // Settle tracking: require SETTLE_HOLD_SEC of continuous in-band time
    if (Math.abs(state.theta2 - STEP_TARGET_RAD) <= TOLERANCE_RAD) {
      continuousSettledSteps++;
      if (continuousSettledSteps >= settleHoldSteps && settledTimeS === -1) {
        settledTimeS = t - SETTLE_HOLD_SEC; // report when it first entered the band
      }
    } else {
      continuousSettledSteps = 0;
      settledTimeS = -1; // band was exited; must re-settle
    }
  }

  return { settledTimeS, maxOvershootRad, finalTheta2: state.theta2 };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('Control — PID step response', () => {
  /**
   * Test 1: Settle time.
   * The body must enter and remain within ±2° of the 45° target for at least
   * SETTLE_HOLD_SEC seconds, within a 30-second window.
   * Failure here = controller too slow, or critically damped and can't reach setpoint.
   */
  it('settles within ±2° of 45° target and holds for ≥2 s, within 30 s (R3.2)', () => {
    const { settledTimeS } = runStepScenario(30, /* seed */ 42);

    expect(settledTimeS, 'Body never settled or exited the band permanently').toBeGreaterThanOrEqual(0);
    expect(settledTimeS, 'Settled too late').toBeLessThan(30 - SETTLE_HOLD_SEC);
  });

  /**
   * Test 2: Overshoot bound.
   * Any overshoot above the 45° setpoint must be ≤ 20% of the step size (≤ 9°).
   * Failure here = controller too aggressive / underdamped.
   */
  it('overshoot does not exceed 20% of step magnitude (≤9° for a 45° step) (R3.2)', () => {
    const { maxOvershootRad } = runStepScenario(30, /* seed */ 42);

    const maxAllowedRad = MAX_OVERSHOOT_FRACTION * STEP_TARGET_RAD; // 20% of π/4 ≈ 0.157 rad
    expect(maxOvershootRad, `Overshoot ${(maxOvershootRad * 180 / Math.PI).toFixed(2)}° exceeds limit`)
      .toBeLessThanOrEqual(maxAllowedRad);
  });
});
