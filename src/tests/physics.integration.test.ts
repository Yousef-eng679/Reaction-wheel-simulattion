/**
 * physics.integration.test.ts
 *
 * Basic physics integration sanity tests.
 *
 * Tests (PRD §6.7, Phase 1 deliverable):
 *   With a known constant motor torque applied for a known duration,
 *   verify that the resulting angular velocities match the analytical closed-form:
 *     ω₁(t) = (τ/I₁) · t        (wheel accelerates)
 *     ω₂(t) = −(τ/I₂) · t       (body decelerates / counter-rotates)
 *   (zero initial velocity, zero friction, zero disturbance)
 *
 * Phase 0: test structure scaffold only.
 *   - Imports are correct.
 *   - describe/it structure is in place.
 *   - Test bodies verify the stub throws 'not implemented' (expected failure at Phase 0).
 *
 * Phase 1 will replace the stub checks with real analytical comparisons.
 */

import { describe, it, expect } from 'vitest';
import { integrate } from '../core/physics/physicsEngine.ts';
import { createZeroState } from '../core/physics/rigidBodyState.ts';

describe('Physics — numerical integration accuracy', () => {
  it('matches analytical solution for constant torque over short duration (no friction)', () => {
    // Phase 0 scaffold: will throw 'not implemented' until Phase 1.
    const state = createZeroState();
    const params = {
      I1: 1e-4,
      I2: 5e-3,
      frictionCoeff1: 0,
      frictionCoeff2: 0,
    };
    const motorTorque = 0.01; // [N·m]
    const dt = 1e-3;          // [s]

    expect(() => {
      integrate(state, motorTorque, 0, dt, params);
    }).toThrow('not implemented');
  });

  it('correctly integrates zero torque (state remains unchanged)', () => {
    const state = createZeroState();
    const params = {
      I1: 1e-4,
      I2: 5e-3,
      frictionCoeff1: 0,
      frictionCoeff2: 0,
    };

    expect(() => {
      integrate(state, 0, 0, 1e-3, params);
    }).toThrow('not implemented');
  });
});
