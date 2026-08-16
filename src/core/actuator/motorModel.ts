/**
 * motorModel.ts
 *
 * Actuator model: simulated BLDC/spindle motor.
 *
 * Models realistic constraints (PRD §6.2):
 *   - Maximum RPM (configurable)
 *   - First-order lag toward target speed (time constant τ)
 *   - Torque saturation (max angular acceleration cap)
 *   - Output: actual torque [N·m] this timestep (not commanded value)
 *
 * R1.3 — Internally operates in SI units (rad/s, N·m). RPM conversion is only
 *         done at input/output display boundaries, never inside the model math.
 * R1.4 — All parameters are named constants or configurable fields, no magic numbers.
 * R2.2 — This module has no imports from ui/, sim/ orchestration.
 *
 * Phase 0: interfaces and signatures only. Bodies throw 'not implemented'.
 * Phase 2 will provide the full implementation.
 */

import type { PhysicsParams } from '../physics/rigidBodyState.ts';

/**
 * Configuration parameters for the motor model.
 * All values in SI units except maxRPM (which is the natural unit for motor spec sheets;
 * the model converts to rad/s internally — display boundary is at input).
 */
export interface MotorParams {
  /**
   * Maximum motor speed [rev/min].
   * PLACEHOLDER — representative of HDD spindle (5400–7200 RPM range).
   * Must be replaced with measured value from hardware characterization. (R1.4)
   */
  maxRPM: number;

  /**
   * Maximum torque the motor can produce [N·m].
   * PLACEHOLDER — representative estimate for a small BLDC/spindle motor. (R1.4)
   */
  maxTorqueNm: number;

  /**
   * First-order lag time constant [s].
   * Models the mechanical + electrical response delay.
   * PLACEHOLDER — requires step-response characterization from real hardware. (R1.4)
   */
  timeConstantS: number;
}

/**
 * MotorModel interface — the public contract for all motor model implementations.
 * Designed so an alternative motor model (e.g. with back-EMF curve) can be swapped in
 * without touching the simulation loop (PRD §6.5 extensibility hook).
 */
export interface MotorModel {
  /**
   * Advances the motor state by one timestep and returns actual torque this step.
   *
   * @param commandedDutyCycle  Commanded duty cycle in range [−1, 1], where
   *                             ±1 = full forward/reverse, 0 = coast.
   *                             (−1 to +1 maps to −maxRPM to +maxRPM target speed)
   * @param dt                  Timestep [s].
   * @returns                   Actual torque delivered this step [N·m], after applying
   *                             first-order lag and saturation. This is what the physics
   *                             engine receives — not the commanded value.
   */
  update(commandedDutyCycle: number, dt: number): number;

  /** Returns the current actual wheel angular velocity [rad/s] (motor internal state). */
  getCurrentOmega(): number;

  /** Resets the motor to zero speed (used on simulation reset). */
  reset(): void;
}

/**
 * Default motor model implementation with first-order lag and torque saturation.
 *
 * Implements MotorModel. Stateful — owns the current wheel speed estimate.
 *
 * Note: the wheel speed tracked here (currentOmega) is the *motor's internal
 * speed estimate*, driven by the commanded duty cycle and lag. The physics engine
 * independently integrates actual wheel angular momentum. In Phase 4, the
 * SimulationLoop will need to reconcile these — see simulationLoop.ts notes.
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 2.
 */
export class DefaultMotorModel implements MotorModel {
  private readonly params: MotorParams;
  // Used in PhysicsParams signature matching — inertias available to motor if needed.
  private readonly _physicsParams: PhysicsParams;

  constructor(params: MotorParams, physicsParams: PhysicsParams) {
    this.params = params;
    this._physicsParams = physicsParams;
    // Suppress unused-var lint in Phase 0 stub
    void this.params;
    void this._physicsParams;
  }

  update(_commandedDutyCycle: number, _dt: number): number {
    throw new Error('not implemented');
  }

  getCurrentOmega(): number {
    throw new Error('not implemented');
  }

  reset(): void {
    throw new Error('not implemented');
  }
}
