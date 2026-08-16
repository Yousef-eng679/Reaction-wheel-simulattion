/**
 * physicsEngine.ts
 *
 * Physics engine: RK4 integrator for the two-body rotational system.
 *
 * Governing equations (PRD §6.1):
 *   I₁ · dω₁/dt =  τ_motor − b₁·ω₁              (wheel: motor torque − friction)
 *   I₂ · dω₂/dt = −τ_motor + τ_dist − b₂·ω₂      (body: reaction + disturbance − friction)
 *   dθ₁/dt = ω₁
 *   dθ₂/dt = ω₂
 *
 * Conservation proof (R1.1):
 *   d/dt (I₁ω₁ + I₂ω₂) = (τ − b₁ω₁) + (−τ + τ_dist − b₂ω₂)
 *                        = τ_dist − b₁ω₁ − b₂ω₂
 *   → With zero friction and zero disturbance, dL/dt = 0 identically at every
 *     RK4 stage, so L is conserved to floating-point machine precision — not just
 *     within 1e-6, but to ~1e-15 relative error. The 1e-6 tolerance in the test
 *     is a deliberately conservative bound.
 *
 * R1.1 — ω₂ is NEVER computed from the algebraic shortcut ω₂ = -(I₁/I₂)·ω₁.
 *         ω₂ emerges purely from integrating its own ODE over time, step by step.
 * R1.2 — 4th-order Runge-Kutta used throughout. No Euler step anywhere.
 * R1.3 — All quantities in SI units (rad, rad/s, N·m, kg·m², s). No RPM/degrees.
 */

import type { RigidBodyState, PhysicsParams } from './rigidBodyState.ts';

// ---------------------------------------------------------------------------
// Internal helpers (not exported — implementation detail of the RK4 scheme)
// ---------------------------------------------------------------------------

/**
 * Computes the time-derivative of the full state vector at the given state.
 * This is the function f(x) in the ODE  dx/dt = f(x).
 *
 * Equations implemented exactly as specified in PRD §6.1:
 *   dθ₁/dt = ω₁
 *   dω₁/dt = (τ_motor  − b₁·ω₁) / I₁
 *   dθ₂/dt = ω₂
 *   dω₂/dt = (−τ_motor + τ_dist − b₂·ω₂) / I₂
 *
 * The motor torque appears with opposite sign in the two ω equations — this is
 * what enforces conservation of angular momentum (action-reaction pair).
 */
function computeDerivatives(
  state: RigidBodyState,
  motorTorque: number,
  disturbanceTorque: number,
  params: PhysicsParams,
): RigidBodyState {
  const { I1, I2, frictionCoeff1, frictionCoeff2 } = params;
  return {
    // dθ₁/dt = ω₁
    theta1: state.omega1,
    // dω₁/dt = (τ_motor − b₁·ω₁) / I₁
    omega1: (motorTorque - frictionCoeff1 * state.omega1) / I1,
    // dθ₂/dt = ω₂
    theta2: state.omega2,
    // dω₂/dt = (−τ_motor + τ_dist − b₂·ω₂) / I₂
    omega2: (-motorTorque + disturbanceTorque - frictionCoeff2 * state.omega2) / I2,
  };
}

/** Adds two state vectors component-wise: result = a + b. */
function addStates(a: RigidBodyState, b: RigidBodyState): RigidBodyState {
  return {
    theta1: a.theta1 + b.theta1,
    omega1: a.omega1 + b.omega1,
    theta2: a.theta2 + b.theta2,
    omega2: a.omega2 + b.omega2,
  };
}

/** Scales a state vector by a scalar: result = s · scalar. */
function scaleState(s: RigidBodyState, scalar: number): RigidBodyState {
  return {
    theta1: s.theta1 * scalar,
    omega1: s.omega1 * scalar,
    theta2: s.theta2 * scalar,
    omega2: s.omega2 * scalar,
  };
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * Advances the system state by one timestep `dt` using 4th-order Runge-Kutta.
 *
 * RK4 formula:
 *   k₁ = f(xₙ)
 *   k₂ = f(xₙ + k₁·dt/2)
 *   k₃ = f(xₙ + k₂·dt/2)
 *   k₄ = f(xₙ + k₃·dt)
 *   x_{n+1} = xₙ + (k₁ + 2k₂ + 2k₃ + k₄) · dt/6
 *
 * Motor torque is treated as constant over the timestep (zero-order hold on
 * the command). This is appropriate for a 1000 Hz physics rate: the control
 * loop never changes the command more than once per millisecond, so the
 * assumption introduces negligible error.
 *
 * R1.1 — ω₂ is integrated from its own ODE; it is never set algebraically.
 * R1.2 — Standard RK4 is implemented in full; no Euler fallback.
 * R1.3 — All parameters and return values in SI units.
 *
 * @param state             Current rigid-body state [rad, rad/s].
 * @param motorTorque       Actual (post-saturation) motor torque [N·m] this step.
 *                          Applied +τ to wheel, −τ to body (conserves momentum).
 * @param disturbanceTorque External torque applied to the body only [N·m].
 *                          Zero for undisturbed runs; nonzero for disturbance tests.
 * @param dt                Integration timestep [s]. Must be PHYSICS_DT_S (R2.5).
 * @param params            Physical parameters (inertias, friction coefficients).
 * @returns                 New rigid-body state after the timestep.
 */
export function integrate(
  state: RigidBodyState,
  motorTorque: number,
  disturbanceTorque: number,
  dt: number,
  params: PhysicsParams,
): RigidBodyState {
  // Stage 1: derivative at the current state
  const k1 = computeDerivatives(state, motorTorque, disturbanceTorque, params);

  // Stage 2: derivative at the midpoint using k1 slope
  const k2 = computeDerivatives(
    addStates(state, scaleState(k1, dt / 2)),
    motorTorque,
    disturbanceTorque,
    params,
  );

  // Stage 3: derivative at the midpoint using k2 slope (better midpoint estimate)
  const k3 = computeDerivatives(
    addStates(state, scaleState(k2, dt / 2)),
    motorTorque,
    disturbanceTorque,
    params,
  );

  // Stage 4: derivative at the endpoint using k3 slope
  const k4 = computeDerivatives(
    addStates(state, scaleState(k3, dt)),
    motorTorque,
    disturbanceTorque,
    params,
  );

  // Weighted combination: (k1 + 2k2 + 2k3 + k4) · dt/6
  const weightedSlope = scaleState(
    addStates(
      addStates(k1, scaleState(k2, 2)),
      addStates(scaleState(k3, 2), k4),
    ),
    dt / 6,
  );

  return addStates(state, weightedSlope);
}

/**
 * Computes total angular momentum of the system [kg·m²·rad/s].
 *
 *   L_total = I₁·ω₁ + I₂·ω₂
 *
 * This is the conserved quantity. With zero friction and zero external disturbance,
 * L_total must remain constant. The conservation invariant test (PRD §6.7) asserts
 * this to within 1e-6 relative tolerance; in practice RK4 achieves ~1e-15 because
 * the motor torque cancels exactly in the derivative at every stage.
 *
 * @param state   Current rigid-body state.
 * @param params  Physical parameters (I₁ and I₂ are used; friction not needed here).
 * @returns       Total angular momentum [kg·m²·rad/s].
 */
export function totalAngularMomentum(
  state: RigidBodyState,
  params: PhysicsParams,
): number {
  return params.I1 * state.omega1 + params.I2 * state.omega2;
}
