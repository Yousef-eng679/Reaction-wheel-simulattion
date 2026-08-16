/**
 * actuator.motorModel.test.ts
 *
 * Unit tests for DefaultMotorModel (Phase 2).
 *
 * Two categories:
 *   1. Saturation — output torque must never exceed maxTorqueNm regardless of
 *      commanded duty cycle. The system must saturate gracefully.
 *   2. Lag — with commands small enough that the torque cap is NOT binding,
 *      the motor omega must follow the exact first-order lag trajectory:
 *        ω(t) = ω_target × (1 − exp(−t / τ))
 *
 * These tests are isolated: no physics engine, no sensor, no controller.
 * The motor model is exercised by calling update() in a loop, which is exactly
 * how Phase 4's SimulationLoop will drive it.
 */

import { describe, it, expect } from 'vitest';
import { DefaultMotorModel } from '../core/actuator/motorModel.ts';
import {
  DEFAULT_MOTOR_MAX_RPM,
  DEFAULT_MOTOR_MAX_TORQUE_NM,
  DEFAULT_MOTOR_TIME_CONSTANT_S,
  DEFAULT_I1_WHEEL_KGM2,
  DEFAULT_I2_BODY_KGM2,
  DEFAULT_FRICTION_COEFF_1,
  DEFAULT_FRICTION_COEFF_2,
  PHYSICS_DT_S,
} from '../core/physics/constants.ts';

// ---------------------------------------------------------------------------
// Shared parameters (all labeled per R1.4)
// ---------------------------------------------------------------------------

const MOTOR_PARAMS = {
  maxRPM: DEFAULT_MOTOR_MAX_RPM,               // [rev/min] PLACEHOLDER
  maxTorqueNm: DEFAULT_MOTOR_MAX_TORQUE_NM,    // [N·m] PLACEHOLDER
  timeConstantS: DEFAULT_MOTOR_TIME_CONSTANT_S, // [s] PLACEHOLDER
};

const PHYSICS_PARAMS = {
  I1: DEFAULT_I1_WHEEL_KGM2,     // [kg·m²] PLACEHOLDER
  I2: DEFAULT_I2_BODY_KGM2,      // [kg·m²] PLACEHOLDER
  frictionCoeff1: DEFAULT_FRICTION_COEFF_1,
  frictionCoeff2: DEFAULT_FRICTION_COEFF_2,
};

const DT = PHYSICS_DT_S; // [s] — 1e-3 s (1 kHz physics rate)

// ---------------------------------------------------------------------------
// Saturation tests
// ---------------------------------------------------------------------------

describe('Actuator — motor model saturation', () => {
  /**
   * Full duty cycle command: the motor wants to accelerate from 0 to maxRPM
   * immediately. The torque needed for this exceeds maxTorqueNm, so saturation
   * must clamp every step.
   *
   * Assert: |torque| ≤ maxTorqueNm at every single step for 2 seconds.
   */
  it('never exceeds maxTorqueNm under full forward duty cycle (dutyCycle = 1)', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);
    const durationSec = 2.0;
    const nSteps = Math.round(durationSec / DT);

    for (let i = 0; i < nSteps; i++) {
      const torque = motor.update(1.0, DT);
      expect(Math.abs(torque)).toBeLessThanOrEqual(MOTOR_PARAMS.maxTorqueNm + 1e-12);
    }
  });

  /**
   * Full reverse duty cycle: same as above but backward.
   * Torque must be ≤ maxTorqueNm in magnitude, and negative.
   */
  it('never exceeds maxTorqueNm under full reverse duty cycle (dutyCycle = −1)', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);
    const nSteps = 2000;

    for (let i = 0; i < nSteps; i++) {
      const torque = motor.update(-1.0, DT);
      expect(Math.abs(torque)).toBeLessThanOrEqual(MOTOR_PARAMS.maxTorqueNm + 1e-12);
    }
  });

  /**
   * Out-of-range duty cycle must be clamped, not cause unbounded torque.
   */
  it('clamps duty cycle > 1 and still respects torque limit', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);
    for (let i = 0; i < 500; i++) {
      const torque = motor.update(5.0, DT); // duty cycle wildly out of range
      expect(Math.abs(torque)).toBeLessThanOrEqual(MOTOR_PARAMS.maxTorqueNm + 1e-12);
    }
  });

  /**
   * Zero command: no torque should be produced from rest.
   */
  it('produces zero torque when commanded zero from rest', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);
    const torque = motor.update(0.0, DT);
    expect(torque).toBe(0);
  });

  /**
   * After spin-up, commanding zero should produce negative torque (braking)
   * but still within the ±maxTorqueNm limit.
   */
  it('braking torque stays within limit when commanding 0 from spinning state', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);
    // Spin up for 1 second at full throttle
    for (let i = 0; i < 1000; i++) motor.update(1.0, DT);

    // Now command stop
    for (let i = 0; i < 500; i++) {
      const torque = motor.update(0.0, DT);
      expect(Math.abs(torque)).toBeLessThanOrEqual(MOTOR_PARAMS.maxTorqueNm + 1e-12);
    }
  });
});

// ---------------------------------------------------------------------------
// Lag behaviour tests
// ---------------------------------------------------------------------------

describe('Actuator — motor model first-order lag', () => {
  /**
   * Choose a duty cycle small enough that the torque cap is NOT binding,
   * so the lag dynamics are visible.
   *
   * Analysis:
   *   maxOmega = 5400 × 2π/60 ≈ 565.5 rad/s
   *   dutyCycle = 0.04 → ω_target = 22.6 rad/s
   *   Lag step (dt=1ms, τ=0.1s): Δω_lag ≈ 22.6 × (1−exp(−0.01)) ≈ 0.224 rad/s
   *   Torque for that: I₁ × 0.224 / 0.001 = 1e-4 × 224 = 0.0224 N·m < maxTorque (0.05 N·m) ✓
   *
   * Expected at t = τ = 0.1 s (100 steps):
   *   ω(τ) = ω_target × (1 − exp(−1)) ≈ ω_target × 0.6321
   */
  it('omega follows first-order exponential: reaches 63.2% of target at t = τ', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);

    const DUTY_CYCLE = 0.04; // small enough for lag to be the binding constraint
    const MAX_OMEGA = MOTOR_PARAMS.maxRPM * (2 * Math.PI / 60); // [rad/s]
    const omegaTarget = DUTY_CYCLE * MAX_OMEGA; // ≈ 22.6 rad/s

    const tau = MOTOR_PARAMS.timeConstantS; // 0.1 s
    const nStepsToTau = Math.round(tau / DT); // 100 steps

    for (let i = 0; i < nStepsToTau; i++) motor.update(DUTY_CYCLE, DT);

    const expectedAt1Tau = omegaTarget * (1 - Math.exp(-1)); // ≈ 14.3 rad/s
    const actual = motor.getCurrentOmega();

    // Tolerance: 1% of target (numerical lag discretisation is tight at 100 steps/τ)
    const tolerance = 0.01 * omegaTarget;
    expect(Math.abs(actual - expectedAt1Tau)).toBeLessThan(tolerance);
  });

  /**
   * At t = 3τ, omega should be ≥ 95% of target (1 − exp(−3) ≈ 0.950).
   * This confirms the motor keeps accelerating toward the target, not stalling.
   */
  it('omega reaches ≥ 95% of target at t = 3τ', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);

    const DUTY_CYCLE = 0.04;
    const MAX_OMEGA = MOTOR_PARAMS.maxRPM * (2 * Math.PI / 60);
    const omegaTarget = DUTY_CYCLE * MAX_OMEGA;

    const tau = MOTOR_PARAMS.timeConstantS;
    const nSteps = Math.round(3 * tau / DT); // 300 steps

    for (let i = 0; i < nSteps; i++) motor.update(DUTY_CYCLE, DT);

    const expected95pct = omegaTarget * (1 - Math.exp(-3));
    expect(motor.getCurrentOmega()).toBeGreaterThan(expected95pct * 0.99);
  });

  /**
   * After reset(), omega returns to zero and the next update behaves as if fresh.
   */
  it('reset() clears internal omega — motor restarts from rest', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);

    // Spin up
    for (let i = 0; i < 500; i++) motor.update(1.0, DT);
    expect(motor.getCurrentOmega()).toBeGreaterThan(0);

    // Reset
    motor.reset();
    expect(motor.getCurrentOmega()).toBe(0);

    // First update from rest should produce a positive torque (lag toward target)
    const torque = motor.update(0.5, DT);
    expect(torque).toBeGreaterThan(0);
  });

  /**
   * With zero command, omega should decay toward zero from a nonzero initial speed.
   * This confirms the lag model also handles deceleration correctly.
   */
  it('decelerates toward zero with zero command (first-order decay)', () => {
    const motor = new DefaultMotorModel(MOTOR_PARAMS, PHYSICS_PARAMS);

    // Pre-spin to a known speed using a small duty cycle (lag-limited)
    const DUTY_CYCLE = 0.02;
    const MAX_OMEGA = MOTOR_PARAMS.maxRPM * (2 * Math.PI / 60);
    const omegaTarget = DUTY_CYCLE * MAX_OMEGA; // ≈ 11.3 rad/s
    const tau = MOTOR_PARAMS.timeConstantS;

    // Run 5τ so motor is near target (≈ 99.3% of target)
    for (let i = 0; i < Math.round(5 * tau / DT); i++) motor.update(DUTY_CYCLE, DT);

    const omegaAtStart = motor.getCurrentOmega();
    // Sanity: close to target
    expect(Math.abs(omegaAtStart - omegaTarget) / omegaTarget).toBeLessThan(0.01);

    // Now command zero — motor should decay exponentially
    for (let i = 0; i < Math.round(tau / DT); i++) motor.update(0.0, DT);

    // After one τ from zero command, omega should be ≈ omegaAtStart × exp(−1) ≈ 36.8%
    const expected = omegaAtStart * Math.exp(-1);
    const actual = motor.getCurrentOmega();
    const tolerance = 0.02 * omegaAtStart; // 2% tolerance

    expect(Math.abs(actual - expected)).toBeLessThan(tolerance);
  });
});
