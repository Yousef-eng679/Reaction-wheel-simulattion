/**
 * physicsEngine.ts
 *
 * Physics engine: RK4 integrator for the two-body rotational system.
 *
 * Governing equations (PRD §6.1):
 *   I₁ · dω₁/dt = τ_motor − b₁·ω₁          (wheel: motor torque + friction)
 *   I₂ · dω₂/dt = −τ_motor + τ_dist − b₂·ω₂  (body: reaction + disturbance + friction)
 *   dθ₁/dt = ω₁
 *   dθ₂/dt = ω₂
 *
 * R1.1 — ω₂ is NEVER computed from an algebraic shortcut like ω₂ = -(I₁/I₂)·ω₁.
 *         It must always emerge from integrating the torque equations over time.
 * R1.2 — Integration uses 4th-order Runge-Kutta (RK4). Plain Euler is forbidden.
 * R1.3 — All quantities in SI units. No RPM/degrees inside this module.
 *
 * Phase 0: signatures and interfaces only. Bodies throw 'not implemented'.
 * Phase 1 will replace the stub with the real RK4 implementation.
 */

import type { RigidBodyState, PhysicsParams } from './rigidBodyState.ts';

/**
 * Advances the system state by one timestep `dt` using 4th-order Runge-Kutta.
 *
 * @param state           Current rigid-body state [rad, rad/s].
 * @param motorTorque     Actual (post-saturation) motor torque [N·m] this step.
 *                        Applied +τ to wheel, −τ to body (conservation).
 * @param disturbanceTorque External torque applied to the body only [N·m].
 *                          Zero for undisturbed runs; nonzero for kick/disturbance tests.
 * @param dt              Integration timestep [s]. Must equal PHYSICS_DT_S (fixed step, R2.5).
 * @param params          Physical parameters (inertias, friction coefficients).
 * @returns               New rigid-body state after the timestep.
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 1.
 */
export function integrate(
  _state: RigidBodyState,
  _motorTorque: number,
  _disturbanceTorque: number,
  _dt: number,
  _params: PhysicsParams,
): RigidBodyState {
  throw new Error('not implemented');
}

/**
 * Computes total angular momentum of the system [kg·m²·rad/s].
 * L_total = I₁·ω₁ + I₂·ω₂
 *
 * This is the conserved quantity tested in physics.conservation.test.ts:
 * with zero friction and zero disturbance, L_total must remain constant
 * to within 1e-6 relative tolerance across arbitrarily long runs.
 *
 * @param state   Current rigid-body state.
 * @param params  Physical parameters (inertias).
 * @returns       Total angular momentum [kg·m²·rad/s].
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 1.
 */
export function totalAngularMomentum(
  _state: RigidBodyState,
  _params: PhysicsParams,
): number {
  throw new Error('not implemented');
}
