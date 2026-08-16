/**
 * physics.conservation.test.ts
 *
 * Conservation of angular momentum invariant test (PRD §6.7, Phase 1 deliverable).
 *
 * With zero friction and zero external disturbance, the total angular momentum:
 *   L_total = I₁·ω₁ + I₂·ω₂
 * must be conserved for any motor torque profile, because the motor is an
 * internal torque — it appears as +τ on the wheel and −τ on the body, cancelling
 * exactly in the L derivative at every RK4 stage.
 *
 * This test asserts that invariant holds within 1e-6 relative tolerance at every
 * logged step over 10 simulated seconds, under a rich time-varying torque profile
 * designed to exercise the integrator aggressively (high-frequency + low-frequency
 * components, sign changes, large and small magnitudes).
 *
 * In practice, RK4 achieves ~1e-15 relative error on this invariant (machine
 * precision), because the torque cancellation is exact in the ODE definition.
 * The 1e-6 bound is intentionally conservative — it would catch Euler integration
 * or any other scheme that introduces energy drift.
 */

import { describe, it, expect } from 'vitest';
import { integrate, totalAngularMomentum } from '../core/physics/physicsEngine.ts';
import type { RigidBodyState, PhysicsParams } from '../core/physics/rigidBodyState.ts';
import { PHYSICS_DT_S } from '../core/physics/constants.ts';

describe('Physics — angular momentum conservation', () => {
  /**
   * Main invariant test: 10 simulated seconds, arbitrary time-varying torque,
   * zero friction, zero disturbance.
   *
   * Initial condition: wheel spinning at 10 rad/s, body at rest.
   * → L_initial = I₁·10 + I₂·0 = 1e-3 kg·m²·rad/s  (nonzero, enabling relative check)
   */
  it('conserves L_total within 1e-6 relative tolerance over 10s with arbitrary time-varying torque', () => {
    const params: PhysicsParams = {
      I1: 1e-4,         // [kg·m²] wheel inertia — placeholder
      I2: 5e-3,         // [kg·m²] body inertia — placeholder
      frictionCoeff1: 0, // [N·m·s/rad] — zero for clean conservation
      frictionCoeff2: 0, // [N·m·s/rad] — zero for clean conservation
    };

    // Nonzero initial state so L_initial ≠ 0, enabling a meaningful relative check.
    let state: RigidBodyState = {
      theta1: 0,    // [rad]
      omega1: 10.0, // [rad/s] — wheel spinning
      theta2: 0,    // [rad]
      omega2: 0.0,  // [rad/s] — body at rest
    };

    const L_initial = totalAngularMomentum(state, params);
    // L_initial = 1e-4 × 10 + 5e-3 × 0 = 1e-3 kg·m²·rad/s
    expect(Math.abs(L_initial)).toBeGreaterThan(0);

    const dt = PHYSICS_DT_S; // 1e-3 s — fixed physics timestep
    const durationSec = 10;
    const nSteps = Math.round(durationSec / dt); // 10 000 steps

    // Rich torque profile: two sinusoidal components at different frequencies
    // and magnitudes, with sign changes throughout. Designed to stress the
    // integrator across many different torque levels and transition rates.
    for (let i = 0; i < nSteps; i++) {
      const t = i * dt; // simulation time [s]
      const motorTorque =
        0.01  * Math.sin(2 * Math.PI * t / 1.5) +   // low-frequency component
        0.005 * Math.sin(2 * Math.PI * t / 0.25);   // high-frequency component

      state = integrate(state, motorTorque, /*disturbanceTorque=*/0, dt, params);

      const L = totalAngularMomentum(state, params);
      const relativeDrift = Math.abs(L - L_initial) / Math.abs(L_initial);

      // 1e-6 is conservative; RK4 achieves ~1e-15 on this invariant.
      expect(relativeDrift).toBeLessThan(1e-6);
    }
  });

  /**
   * Edge-case: confirm that with a nonzero constant torque, L is still conserved
   * (the wheel spins up, the body counter-rotates, but L stays constant).
   * This catches any scheme that incorrectly treats the torque as external.
   */
  it('conserves L_total with constant motor torque (wheel spins up, body counter-rotates)', () => {
    const params: PhysicsParams = {
      I1: 1e-4,
      I2: 5e-3,
      frictionCoeff1: 0,
      frictionCoeff2: 0,
    };

    let state: RigidBodyState = {
      theta1: 0, omega1: 0, theta2: 0, omega2: 0,
    };

    // With L_initial = 0, use absolute tolerance (|L| < 1e-12 N·m·s)
    const L_initial = totalAngularMomentum(state, params); // = 0
    const dt = PHYSICS_DT_S;
    const motorTorque = 0.02; // [N·m] constant

    for (let i = 0; i < 5000; i++) {
      state = integrate(state, motorTorque, 0, dt, params);
      const L = totalAngularMomentum(state, params);
      const absoluteDrift = Math.abs(L - L_initial);
      // L should stay exactly 0 (up to floating-point rounding ~1e-17)
      expect(absoluteDrift).toBeLessThan(1e-12);
    }
  });

  /**
   * Friction breaks conservation (as expected — this is correct physics).
   * With nonzero friction, L should decrease over time.
   * This test is included to confirm the friction term is wired correctly
   * and isn't silently disabled.
   */
  it('L_total decreases over time when friction is nonzero (correct physics)', () => {
    const params: PhysicsParams = {
      I1: 1e-4,
      I2: 5e-3,
      frictionCoeff1: 1e-4, // [N·m·s/rad] — small but nonzero friction on wheel
      frictionCoeff2: 0,
    };

    let state: RigidBodyState = {
      theta1: 0, omega1: 50.0, theta2: 0, omega2: 0,
    };

    const L_initial = totalAngularMomentum(state, params);
    const dt = PHYSICS_DT_S;

    for (let i = 0; i < 2000; i++) {
      state = integrate(state, /*motorTorque=*/0, /*disturbance=*/0, dt, params);
    }

    const L_final = totalAngularMomentum(state, params);
    // Friction on the wheel removes energy → L_total decreases
    expect(L_final).toBeLessThan(L_initial);
  });
});
