/**
 * control.saturation.test.ts
 *
 * Actuator saturation stability test — system must not diverge under saturation.
 *
 * Tests (PRD §6.7, Phase 3 deliverable):
 *   Command an unreasonably large step target (or configure very low max RPM)
 *   so the actuator saturates immediately. Assert that:
 *     - The closed-loop system does NOT diverge (state values stay bounded).
 *     - Body angle and angular velocity remain within finite numeric bounds.
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

const LARGE_STEP_RAD = 10 * Math.PI * 2; // 10 full rotations [rad]

describe('Control — actuator saturation', () => {
  it('does not diverge under actuator saturation (state stays bounded)', () => {
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
      seed: 99,
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
    const durationS = 60;
    const steps = durationS / dt;
    const sensorTicks = Math.round((1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ) / dt);

    let lastControlTorque = 0;
    let maxAbsTheta2 = 0;
    let maxAbsOmega2 = 0;

    for (let i = 0; i < steps; i++) {
      const reading = sensor.sample(state, dt);
      
      let dutyCycleCommand = lastControlTorque / OUTPUT_MAX;
      
      if (i % sensorTicks === 0) {
        estimator.update(reading, 1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ);
        const outputNm = pid.update(LARGE_STEP_RAD, estimator.getEstimate(), 1 / DEFAULT_SENSOR_SAMPLE_RATE_HZ);
        dutyCycleCommand = -outputNm / OUTPUT_MAX;
      }
      
      lastControlTorque = motor.update(dutyCycleCommand, dt);
      state = integrate(state, lastControlTorque, 0, dt, physicsParams);

      if (Math.abs(state.theta2) > maxAbsTheta2) maxAbsTheta2 = Math.abs(state.theta2);
      if (Math.abs(state.omega2) > maxAbsOmega2) maxAbsOmega2 = Math.abs(state.omega2);
    }

    // Because we use a small max torque and limited RPM, it takes a long time to reach 10 rotations.
    // The point is that it doesn't blow up to infinity.
    // Assert explicit numeric bounds
    expect(maxAbsTheta2).toBeLessThan(LARGE_STEP_RAD * 1.5); // Should not significantly overshoot beyond the huge setpoint
    expect(maxAbsOmega2).toBeLessThan(20); // Should not spin at some insane rate
    
    // Also assert that PID integral did not wind up to infinity
    const finalIntegral = Math.abs(pid.getIntegralTerm());
    expect(finalIntegral).toBeLessThan(OUTPUT_MAX * 10); // Conditional integration should prevent massive windup
  });
});
