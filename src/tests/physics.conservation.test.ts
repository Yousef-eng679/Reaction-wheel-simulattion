/**
 * physics.conservation.test.ts
 *
 * Conservation of angular momentum invariant test.
 *
 * Tests (PRD §6.7, Phase 1 deliverable):
 *   With zero friction (frictionCoeff1 = frictionCoeff2 = 0) and zero disturbance
 *   torque, apply an arbitrary time-varying motor torque profile for 10 simulated
 *   seconds. Assert that:
 *     L_total = I₁·ω₁ + I₂·ω₂
 *   stays within 1e-6 relative tolerance of its t=0 value at EVERY logged step.
 *
 * Phase 0: test structure scaffold only.
 *   - import exists and is correct
 *   - describe/it structure is in place
 *   - test bodies contain an explicit `throw` / `skip` until Phase 1 implements
 *     the physics engine (these are expected failures at this phase)
 *
 * Phase 1 will implement the actual test assertions.
 */

import { describe, it, expect } from 'vitest';
import { integrate, totalAngularMomentum } from '../core/physics/physicsEngine.ts';
import { createZeroState } from '../core/physics/rigidBodyState.ts';

describe('Physics — angular momentum conservation', () => {
  it('conserves L_total within 1e-6 relative tolerance over 10s with zero friction', () => {
    // Phase 0 scaffold: stub throws 'not implemented' — expected to fail until Phase 1.
    // The assertions below will be filled in during Phase 1 implementation.
    const initialState = createZeroState();
    const params = {
      I1: 1e-4,  // [kg·m²] placeholder
      I2: 5e-3,  // [kg·m²] placeholder
      frictionCoeff1: 0,
      frictionCoeff2: 0,
    };

    // This call will throw 'not implemented' in Phase 0 — expected.
    expect(() => {
      integrate(initialState, 0.01, 0, 1e-3, params);
    }).toThrow('not implemented');

    expect(() => {
      totalAngularMomentum(initialState, params);
    }).toThrow('not implemented');
  });
});
