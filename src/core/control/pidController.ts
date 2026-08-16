/**
 * pidController.ts
 *
 * PID controller — the portable core.
 *
 * This is the single most architecture-critical file in the project (PRD §6.4, §8, R2.2).
 * It must be:
 *   - Framework-agnostic: zero imports from ui/, sim/, or any DOM/React APIs.
 *   - Pure: control computation isolated in a single update() method.
 *   - Portable: written to translate to Arduino C++ almost line-for-line.
 *   - Correct: implements the specific PID features required by PRD §6.4 —
 *     integral windup protection and derivative-on-measurement (not derivative-on-error).
 *
 * R2.2 — ZERO imports from ui/, sim/ orchestration, DOM, or async/Promise.
 * R1.3 — All values in SI units (rad, rad/s, N·m). No display-unit conversions here.
 * R2.1 — This module ONLY receives sensor-estimated angle, never ground truth.
 */

export interface PidParams {
  /** Proportional gain [N·m/rad]. */
  kp: number;
  /** Integral gain [N·m/(rad·s)]. */
  ki: number;
  /** Derivative gain [N·m·s/rad]. Applied to measurement, not error. */
  kd: number;
  /** Minimum controller output [N·m]. Used for output clamping and windup protection. */
  outputMin: number;
  /** Maximum controller output [N·m]. Used for output clamping and windup protection. */
  outputMax: number;
}

/**
 * PidController — standard PID with correct real-world features.
 *
 * Features implemented:
 *   1. Integral windup protection via back-calculation: integral accumulation halts
 *      when the output is saturated.
 *   2. Derivative-on-measurement: derivative is computed from the estimated angle
 *      signal (not error), preventing derivative kick on setpoint step changes.
 *   3. Output clamping: output is strictly clamped to [outputMin, outputMax].
 *   4. Live gain changes supported via setGains().
 */
export class PidController {
  private params: PidParams;

  // Internal state
  private integralTerm: number = 0;
  private lastMeasurement: number = 0;
  private hasLastMeasurement: boolean = false;
  private lastRawOutput: number = 0;

  constructor(params: PidParams) {
    this.params = { ...params };
  }

  /**
   * Computes PID output for one control step.
   *
   * @param setpoint     Target body angle [rad].
   * @param measurement  Estimated body angle from the sensor/estimator [rad]. (Never ground truth!)
   * @param dt           Elapsed time since last update [s].
   * @returns            Control output [N·m], clamped to [outputMin, outputMax].
   */
  update(setpoint: number, measurement: number, dt: number): number {
    if (dt <= 0) return 0; // Guard against zero/negative dt

    const error = setpoint - measurement;

    // Proportional term
    const pTerm = this.params.kp * error;

    // Derivative on measurement (to avoid derivative kick)
    let dMeasurement = 0;
    if (this.hasLastMeasurement) {
      dMeasurement = (measurement - this.lastMeasurement) / dt;
    } else {
      this.hasLastMeasurement = true;
    }
    this.lastMeasurement = measurement;

    // Note: dMeasurement replaces dError. The standard dError/dt is d(setpoint - measurement)/dt.
    // Assuming setpoint is constant, dError/dt = -dMeasurement/dt.
    // So the D term is -kd * dMeasurement.
    const dTerm = -this.params.kd * dMeasurement;

    // Calculate integral term (tentative)
    // We add to the integral term here, but we will back-calculate if saturated.
    const iTermUnbounded = this.integralTerm + (this.params.ki * error * dt);

    // Raw unbounded output
    const rawOutput = pTerm + iTermUnbounded + dTerm;
    this.lastRawOutput = rawOutput;

    // Clamped output
    let clampedOutput = rawOutput;
    if (clampedOutput > this.params.outputMax) {
      clampedOutput = this.params.outputMax;
    } else if (clampedOutput < this.params.outputMin) {
      clampedOutput = this.params.outputMin;
    }

    // Integral windup protection (back-calculation)
    // If saturated, we adjust the integral term so that the raw output equals the clamped output.
    // This provides a smooth exit from saturation.
    // i_new = clamped_output - p_term - d_term
    // Wait, standard back-calculation / clamping logic:
    if (rawOutput !== clampedOutput && this.params.ki !== 0) {
      // Actually, a simpler and equally valid anti-windup is conditional integration:
      // only integrate if we are not saturated, OR if integrating helps us un-saturate 
      // (error and saturated output have opposite signs).
      // Here we implement conditional integration (clamping):
      if ((rawOutput > this.params.outputMax && error > 0) || 
          (rawOutput < this.params.outputMin && error < 0)) {
        // Do not accumulate integral (windup limit)
        // Keep integralTerm as it was before this step
      } else {
        this.integralTerm = iTermUnbounded;
      }
    } else {
      this.integralTerm = iTermUnbounded;
    }

    // Recalculate output with the protected integral term to ensure consistency
    const finalRawOutput = pTerm + this.integralTerm + dTerm;
    let finalOutput = finalRawOutput;
    if (finalOutput > this.params.outputMax) finalOutput = this.params.outputMax;
    if (finalOutput < this.params.outputMin) finalOutput = this.params.outputMin;

    return finalOutput;
  }

  /**
   * Live-updates PID gains without resetting integral or derivative state.
   */
  setGains(kp: number, ki: number, kd: number): void {
    this.params.kp = kp;
    this.params.ki = ki;
    this.params.kd = kd;
  }

  /**
   * Resets the controller's internal state.
   */
  reset(): void {
    this.integralTerm = 0;
    this.hasLastMeasurement = false;
    this.lastMeasurement = 0;
    this.lastRawOutput = 0;
  }

  /**
   * Returns the last computed raw PID output before clamping [N·m].
   */
  getLastRawOutput(): number {
    return this.lastRawOutput;
  }

  /**
   * Returns the current integral accumulator value.
   */
  getIntegralTerm(): number {
    return this.integralTerm;
  }
}
