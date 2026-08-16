/**
 * motorModel.ts
 *
 * Actuator model: simulated BLDC/spindle motor (PRD §6.2).
 *
 * Models realistic constraints:
 *   1. Maximum RPM — target speed is capped at ±maxOmegaRad [rad/s].
 *   2. First-order lag — motor doesn't reach target speed instantly.
 *      Uses the exact discrete-time solution to the first-order ODE:
 *        ω(t+dt) = ω_target − (ω_target − ω(t)) · exp(−dt / τ)
 *   3. Torque saturation — the motor cannot produce more than maxTorqueNm [N·m].
 *      The lag step is clamped so |Δω/dt · I₁| ≤ maxTorqueNm.
 *   4. Output — actual torque this step = I₁ · Δω_actual / dt [N·m].
 *      This is what the physics engine receives, after lag and saturation.
 *
 * R1.3 — All math in SI units (rad/s, N·m, kg·m², s).
 *         RPM→rad/s conversion happens once, at construction time. No RPM anywhere else.
 * R1.4 — All parameters are named fields, no magic literals.
 * R2.2 — Zero imports from ui/, sim/, DOM, or framework code.
 *
 * Design note on motor ↔ physics coupling:
 *   The motor model tracks its *internal* speed estimate, driven by the commanded
 *   duty cycle and lag. The physics engine independently integrates actual wheel
 *   angular momentum. In Phase 4 (SimulationLoop), the motor's internal omega is
 *   kept in sync with the physics engine's omega1 each step, so they don't diverge.
 */

import type { PhysicsParams } from '../physics/rigidBodyState.ts';

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
   * Models the combined mechanical + electrical response delay.
   * PLACEHOLDER — requires step-response characterization from real hardware. (R1.4)
   */
  timeConstantS: number;
}

/**
 * MotorModel interface — public contract for all motor model implementations.
 * Behind an interface so an alternative (e.g. back-EMF curve) can be swapped
 * in without touching the simulation loop (PRD §6.5 extensibility hook).
 */
export interface MotorModel {
  /**
   * Advances motor state by one timestep, returns actual torque this step.
   *
   * @param commandedDutyCycle  Duty cycle in [−1, 1]: ±1 = ±maxRPM target, 0 = coast.
   * @param dt                  Timestep [s].
   * @returns                   Actual torque [N·m] after lag and saturation.
   *                             Positive = accelerates wheel (and pushes body backwards).
   */
  update(commandedDutyCycle: number, dt: number): number;

  /** Current internal wheel angular velocity estimate [rad/s]. */
  getCurrentOmega(): number;

  /** Resets motor to zero speed (simulation reset). */
  reset(): void;
}

/**
 * DefaultMotorModel — first-order lag with torque saturation.
 *
 * Step-by-step update logic:
 *   1. Clamp duty cycle to [−1, 1].
 *   2. Compute ω_target = dutyCycle × maxOmegaRad.
 *   3. Compute unconstrained lag step using exact discrete-time solution:
 *        ω_lag = ω_current + (ω_target − ω_current) × (1 − exp(−dt / τ))
 *   4. Compute Δω_unconstrained = ω_lag − ω_current.
 *   5. Compute max allowable Δω from torque cap:
 *        Δω_max = maxTorqueNm × dt / I₁
 *   6. Clamp: Δω_actual = clamp(Δω_unconstrained, −Δω_max, +Δω_max).
 *   7. Update: currentOmega += Δω_actual.
 *   8. Return actualTorque = I₁ × Δω_actual / dt.
 */
export class DefaultMotorModel implements MotorModel {
  private readonly motorParams: MotorParams;
  private readonly physicsParams: PhysicsParams;

  /** Pre-computed max wheel speed [rad/s]. Conversion happens once at construction. */
  private readonly maxOmegaRad: number;

  /** Current internal wheel angular velocity [rad/s]. Motor's own state variable. */
  private currentOmega: number = 0;

  constructor(motorParams: MotorParams, physicsParams: PhysicsParams) {
    this.motorParams = motorParams;
    this.physicsParams = physicsParams;
    // R1.3: convert RPM → rad/s once at construction; never use RPM in math below.
    // 1 rev/min = 2π/60 rad/s
    this.maxOmegaRad = motorParams.maxRPM * (2 * Math.PI / 60);
  }

  /**
   * Advances motor state and returns actual torque [N·m].
   * See class-level doc for the step-by-step logic.
   */
  update(commandedDutyCycle: number, dt: number): number {
    // Step 1: clamp duty cycle to valid range [−1, 1]
    const dutyCycle = Math.max(-1, Math.min(1, commandedDutyCycle));

    // Step 2: target angular velocity [rad/s]
    const omegaTarget = dutyCycle * this.maxOmegaRad;

    // Step 3: exact first-order lag step (avoids Euler drift at large dt/τ ratios).
    // dω/dt = (ω_target − ω) / τ  →  ω(t+dt) = ω_target − (ω_target − ω) · exp(−dt/τ)
    const lagFactor = 1 - Math.exp(-dt / this.motorParams.timeConstantS);
    const omegaLag = this.currentOmega + lagFactor * (omegaTarget - this.currentOmega);

    // Step 4: unconstrained speed change this step
    let deltaOmega = omegaLag - this.currentOmega;

    // Step 5: maximum speed change allowed by torque cap
    // τ_max = I₁ · (dω/dt)_max  →  Δω_max = τ_max · dt / I₁
    const deltaOmegaMax = (this.motorParams.maxTorqueNm * dt) / this.physicsParams.I1;

    // Step 6: clamp to torque limit (saturation)
    deltaOmega = Math.max(-deltaOmegaMax, Math.min(deltaOmegaMax, deltaOmega));

    // Step 7: update internal state
    this.currentOmega += deltaOmega;

    // Step 8: actual torque = I₁ · Δω / dt [N·m]
    // (dt > 0 guaranteed by physics loop; guard against exactly zero just in case)
    const actualTorque = dt > 0 ? (this.physicsParams.I1 * deltaOmega) / dt : 0;

    return actualTorque;
  }

  /** Returns current wheel speed estimate [rad/s]. */
  getCurrentOmega(): number {
    return this.currentOmega;
  }

  /**
   * Resets motor to rest. Call on simulation reset.
   * Optionally accepts a known speed to synchronise with the physics engine
   * (used by SimulationLoop in Phase 4 to prevent internal-state divergence).
   */
  reset(knownOmega: number = 0): void {
    this.currentOmega = knownOmega;
  }
}
