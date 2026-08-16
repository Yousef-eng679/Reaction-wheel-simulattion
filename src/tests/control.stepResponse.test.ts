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

const STEP_TARGET_RAD = Math.PI / 4; // 45°
const TOLERANCE_RAD = (2 * Math.PI) / 180; // ±2°

describe('Control — PID step response', () => {
  it('settles within ±2° of 45° target within 30s with default gains, overshoot <= 20%', () => {
    // Setup closed-loop components
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
      seed: 42,
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
    let maxOvershoot = 0;
    let settledTimeS = -1;
    
    const dt = PHYSICS_DT_S;
    const durationS = 30;
    const steps = durationS / dt;
    const sensorTicks = Math.round((1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ) / dt);

    let lastControlTorque = 0;

    for (let i = 0; i < steps; i++) {
      const t = i * dt;

      // 1. Sensor & Estimator update (runs at sensor rate)
      const reading = sensor.sample(state, dt);
      if (i % sensorTicks === 0) {
        estimator.update(reading, 1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ);
        
        // 2. Control update (using estimate)
        // Convert N*m to duty cycle? The PRD/motorModel expects duty cycle [-1, 1],
        // but PID output is Nm. Actually, PID output *is* the duty cycle or Nm?
        // Wait, the PID output limit is OUTPUT_MAX (which is maxTorqueNm).
        // If output is Nm, we can just pass output / maxTorqueNm as duty cycle.
        // Invert the PID output: positive motor torque applies negative body torque.
        // To move the body in the positive direction (positive error), we need negative motor torque.
        const outputNm = pid.update(STEP_TARGET_RAD, estimator.getEstimate(), 1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ);
        const dutyCycle = -outputNm / OUTPUT_MAX;
        
        lastControlTorque = motor.update(dutyCycle, dt);
      } else {
        // Between sensor ticks, motor still updates
        lastControlTorque = motor.update(lastControlTorque / OUTPUT_MAX, dt);
      }

      // 3. Physics step
      state = integrate(state, lastControlTorque, 0, dt, physicsParams);

      // Analyze metrics
      const error = state.theta2 - STEP_TARGET_RAD;
      if (state.theta2 > STEP_TARGET_RAD) {
        if (state.theta2 - STEP_TARGET_RAD > maxOvershoot) {
          maxOvershoot = state.theta2 - STEP_TARGET_RAD;
        }
      }

      if (Math.abs(error) <= TOLERANCE_RAD) {
        if (settledTimeS === -1) settledTimeS = t;
      } else {
        settledTimeS = -1; // Reset if it goes out of bounds
      }
    }

    // Must settle
    expect(settledTimeS).toBeGreaterThanOrEqual(0);
    expect(settledTimeS).toBeLessThan(30);

    // Overshoot should be <= 20% of the step (20% of 45° is 9°)
    const maxAllowedOvershoot = 0.2 * STEP_TARGET_RAD;
    expect(maxOvershoot).toBeLessThanOrEqual(maxAllowedOvershoot);
    
    // Check final state is within tolerance
    expect(Math.abs(state.theta2 - STEP_TARGET_RAD)).toBeLessThanOrEqual(TOLERANCE_RAD);
  });
});
