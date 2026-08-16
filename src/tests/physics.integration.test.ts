/**
 * physics.integration.test.ts
 *
 * RK4 accuracy tests: compare numerical integration against analytical solutions
 * for simple cases where exact closed-form answers are known (PRD §6.7, Phase 1).
 *
 * Two scenarios are tested:
 *
 * 1. Constant torque, no friction (linear ODE → RK4 gives exact result):
 *    With dω₁/dt = τ/I₁ = constant and dω₂/dt = −τ/I₂ = constant,
 *    all four RK4 stages produce the same derivative, so the weighted average
 *    equals the exact answer. After N steps of dt:
 *      ω₁(T) = (τ/I₁)·T  exactly (to floating-point precision)
 *      ω₂(T) = −(τ/I₂)·T exactly
 *      θ₁(T) = ½·(τ/I₁)·T²  exactly
 *      θ₂(T) = −½·(τ/I₂)·T² exactly
 *
 * 2. Zero torque, zero initial velocity → state is exactly unchanged.
 *
 * 3. Disturbance torque applied to body only (with zero motor torque):
 *    L_total changes at rate τ_dist (correct physics — external torque).
 *    ω₂(T) = (τ_dist / I₂)·T  analytically.
 */

import { describe, it, expect } from 'vitest';
import { integrate, totalAngularMomentum } from '../core/physics/physicsEngine.ts';
import { createZeroState } from '../core/physics/rigidBodyState.ts';
import type { PhysicsParams } from '../core/physics/rigidBodyState.ts';
import { PHYSICS_DT_S } from '../core/physics/constants.ts';

// ---------------------------------------------------------------------------
// Shared parameters (all SI units, all labeled per R1.4)
// ---------------------------------------------------------------------------

const PARAMS: PhysicsParams = {
  I1: 1e-4,         // [kg·m²] wheel inertia — placeholder
  I2: 5e-3,         // [kg·m²] body inertia — placeholder
  frictionCoeff1: 0, // [N·m·s/rad] — zero for analytical comparison
  frictionCoeff2: 0, // [N·m·s/rad] — zero for analytical comparison
};

const DT = PHYSICS_DT_S; // 1e-3 s

describe('Physics — numerical integration accuracy', () => {
  /**
   * Test 1: constant torque over 1 second.
   *
   * With zero friction and constant torque, the ODE has a constant right-hand side,
   * so all four RK4 stages evaluate the same derivative. The weighted average
   * (k1 + 2k2 + 2k3 + k4)/6 is therefore exact — no truncation error whatsoever.
   *
   * Analytical solution (starting from rest):
   *   ω₁(1s) = (0.01 / 1e-4) × 1.0 =  100.0 rad/s
   *   ω₂(1s) = −(0.01 / 5e-3) × 1.0 =  −2.0 rad/s
   *   θ₁(1s) = ½ × (0.01/1e-4) × 1.0² = 50.0 rad
   *   θ₂(1s) = −½ × (0.01/5e-3) × 1.0² = −1.0 rad
   *
   * Expected accuracy: floating-point rounding only (~1e-12 relative),
   * because there is no Taylor-series truncation error in this case.
   */
  it('matches analytical solution for constant torque over 1 second (no friction)', () => {
    let state = createZeroState();
    const motorTorque = 0.01; // [N·m] — constant
    const durationSec = 1.0;
    const nSteps = Math.round(durationSec / DT); // 1 000 steps

    for (let i = 0; i < nSteps; i++) {
      state = integrate(state, motorTorque, /*disturbance=*/0, DT, PARAMS);
    }

    // Analytical angular velocities
    const expectedOmega1 = (motorTorque / PARAMS.I1) * durationSec;   //  100.0 rad/s
    const expectedOmega2 = -(motorTorque / PARAMS.I2) * durationSec;  //   −2.0 rad/s

    // Analytical angles (uniform acceleration from rest: θ = ½ α t²)
    const expectedTheta1 = 0.5 * (motorTorque / PARAMS.I1) * durationSec ** 2;  //  50.0 rad
    const expectedTheta2 = -0.5 * (motorTorque / PARAMS.I2) * durationSec ** 2; //  −1.0 rad

    // Relative error tolerance: 1e-10 (well above floating-point floor of ~1e-15,
    // conservatively allowing for accumulated rounding across 1 000 steps).
    const relTol = 1e-10;

    expect(Math.abs(state.omega1 - expectedOmega1) / Math.abs(expectedOmega1))
      .toBeLessThan(relTol);
    expect(Math.abs(state.omega2 - expectedOmega2) / Math.abs(expectedOmega2))
      .toBeLessThan(relTol);
    expect(Math.abs(state.theta1 - expectedTheta1) / Math.abs(expectedTheta1))
      .toBeLessThan(relTol);
    expect(Math.abs(state.theta2 - expectedTheta2) / Math.abs(expectedTheta2))
      .toBeLessThan(relTol);
  });

  /**
   * Test 2: zero torque, zero initial velocity → state must stay exactly zero.
   *
   * With no torque and no friction and no initial motion, all derivatives are zero,
   * so the RK4 output should be bit-for-bit identical to the input.
   */
  it('state remains exactly zero with zero torque and zero initial velocity', () => {
    const state = createZeroState();
    const newState = integrate(state, /*motorTorque=*/0, /*disturbance=*/0, DT, PARAMS);

    expect(newState.theta1).toBe(0);
    expect(newState.omega1).toBe(0);
    expect(newState.theta2).toBe(0);
    expect(newState.omega2).toBe(0);
  });

  /**
   * Test 3: nonzero initial velocity, zero torque, zero friction.
   *
   * With ω₁₀ and ω₂₀ nonzero and no torques, the body and wheel coast freely.
   * Analytically:
   *   ω₁(T) = ω₁₀  (unchanged — no torque to change it)
   *   ω₂(T) = ω₂₀
   *   θ₁(T) = θ₁₀ + ω₁₀·T
   *   θ₂(T) = θ₂₀ + ω₂₀·T
   */
  it('free-coast: angles advance linearly with constant angular velocity, no torque', () => {
    const omega1_0 = 20.0;  // [rad/s]
    const omega2_0 = -0.4;  // [rad/s]

    let state = {
      theta1: 0.5, omega1: omega1_0, theta2: -0.1, omega2: omega2_0,
    };

    const durationSec = 2.0;
    const nSteps = Math.round(durationSec / DT); // 2 000 steps

    for (let i = 0; i < nSteps; i++) {
      state = integrate(state, 0, 0, DT, PARAMS);
    }

    const expectedTheta1 = 0.5 + omega1_0 * durationSec;   // 0.5 + 40.0
    const expectedTheta2 = -0.1 + omega2_0 * durationSec;  // -0.1 − 0.8
    const relTol = 1e-10;

    expect(Math.abs(state.omega1 - omega1_0) / Math.abs(omega1_0)).toBeLessThan(relTol);
    expect(Math.abs(state.omega2 - omega2_0) / Math.abs(omega2_0)).toBeLessThan(relTol);
    expect(Math.abs(state.theta1 - expectedTheta1) / Math.abs(expectedTheta1)).toBeLessThan(relTol);
    expect(Math.abs(state.theta2 - expectedTheta2) / Math.abs(expectedTheta2)).toBeLessThan(relTol);
  });

  /**
   * Test 4: disturbance torque applied to body only (zero motor torque).
   *
   * τ_dist is external → L_total changes at rate τ_dist (correct physics).
   * Analytically, starting from rest:
   *   ω₁(T) = 0          (no torque on wheel)
   *   ω₂(T) = (τ_dist / I₂) · T
   *   ΔL(T) = τ_dist · T  (momentum added by external torque)
   *
   * This confirms the disturbance path is correctly wired to the body equation
   * and does NOT appear in the wheel equation.
   */
  it('disturbance torque drives body angle correctly without affecting wheel', () => {
    let state = createZeroState();
    const disturbanceTorque = 0.005; // [N·m] external torque on body
    const durationSec = 1.0;
    const nSteps = Math.round(durationSec / DT);

    for (let i = 0; i < nSteps; i++) {
      state = integrate(state, /*motorTorque=*/0, disturbanceTorque, DT, PARAMS);
    }

    // Wheel must be unaffected (zero torque applied to it)
    expect(state.omega1).toBe(0);

    // Body angular velocity: ω₂(T) = (τ_dist / I₂) · T
    const expectedOmega2 = (disturbanceTorque / PARAMS.I2) * durationSec; // 1.0 rad/s
    const relTol = 1e-10;
    expect(Math.abs(state.omega2 - expectedOmega2) / expectedOmega2).toBeLessThan(relTol);

    // Angular momentum should have increased by τ_dist · T
    const L_final = totalAngularMomentum(state, PARAMS);
    const expectedDeltaL = disturbanceTorque * durationSec; // 0.005 × 1.0 = 5e-3
    expect(Math.abs(L_final - expectedDeltaL) / expectedDeltaL).toBeLessThan(relTol);
  });
});
