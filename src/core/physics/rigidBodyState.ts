/**
 * rigidBodyState.ts
 *
 * Ground-truth state vector for the two-body rotational system:
 *   - Body 1: the reaction wheel (rotor)
 *   - Body 2: the spacecraft body (stator/chassis)
 *
 * All angles in radians, all angular velocities in rad/s (SI units — R1.3).
 * This type is the single source of truth for physics state throughout the sim.
 * The UI layer converts to degrees/RPM only at the display boundary.
 */

export interface RigidBodyState {
  /** Wheel angle, θ₁ [rad]. Not used by the controller but tracked for visualization. */
  theta1: number;

  /** Wheel angular velocity, ω₁ [rad/s]. */
  omega1: number;

  /** Body (spacecraft) angle, θ₂ [rad]. This is the controlled output. */
  theta2: number;

  /** Body angular velocity, ω₂ [rad/s]. */
  omega2: number;
}

/**
 * Parameters required by the physics engine for one integration step.
 * All values in SI units (R1.3).
 */
export interface PhysicsParams {
  /** Moment of inertia of the reaction wheel [kg·m²] — placeholder pending hardware characterization (R1.4). */
  I1: number;

  /** Moment of inertia of the spacecraft body [kg·m²] — placeholder pending hardware characterization (R1.4). */
  I2: number;

  /**
   * Bearing/air friction coefficient for the wheel [N·m·s/rad].
   * A small torque proportional to ω₁ opposing wheel rotation.
   * Placeholder estimate — to be replaced with measured values (R1.4).
   */
  frictionCoeff1: number;

  /**
   * Bearing/air friction coefficient for the body [N·m·s/rad].
   * A small torque proportional to ω₂ opposing body rotation.
   * Placeholder estimate — to be replaced with measured values (R1.4).
   */
  frictionCoeff2: number;
}

/** Creates a zero-initialized state (body and wheel at rest, angles at 0). */
export function createZeroState(): RigidBodyState {
  return { theta1: 0, omega1: 0, theta2: 0, omega2: 0 };
}
