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
  /** Derivative gain [N·m·s/rad]. Applied to measurement, not error, to avoid kick. */
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
 *   1. Integral windup protection via conditional integration:
 *      the integral accumulates only when the output is not saturated, OR when
 *      integrating would drive the output back toward the linear region (i.e. error
 *      and saturation direction are opposite). This prevents unlimited wind-up while
 *      still allowing the integral to work the system out of saturation when the
 *      error reverses.
 *   2. Derivative-on-measurement: derivative is computed from the estimated angle
 *      signal (not from error), so a setpoint step change does not produce a
 *      derivative spike (kick). Sign: d/dt(error) = -d/dt(measurement) for
 *      constant setpoint, so dTerm = -kd × (dMeasurement/dt).
 *   3. Output clamping: output is strictly clamped to [outputMin, outputMax].
 *   4. Live gain changes supported via setGains() — applied on the next update().
 *
 * Portability note: the update() logic translates to embedded C++ almost line-for-line.
 * All state is primitive scalars; no dynamic allocation, no closures, no JS-specifics.
 */
export class PidController {
  private params: PidParams;

  // Internal state
  private integralTerm: number = 0;
  private lastMeasurement: number = 0;
  private hasLastMeasurement: boolean = false;

  /**
   * The final pre-clamp output from the last update() call [N·m].
   * Exposed via getLastRawOutput() for telemetry (saturation visualization in Phase 5).
   * Stored AFTER windup protection and integral recalculation, so it accurately
   * represents the controller's intention before the actuator limit is applied.
   */
  private lastRawOutput: number = 0;

  constructor(params: PidParams) {
    this.params = { ...params };
  }

  /**
   * Computes PID output for one control step.
   *
   * Algorithm (step by step, portable to C++):
   *   1. error = setpoint − measurement
   *   2. pTerm = kp × error
   *   3. dMeasurement = (measurement − lastMeasurement) / dt   [first step = 0]
   *      dTerm = −kd × dMeasurement                           [sign: see class doc]
   *   4. Tentative integral = integralTerm + ki × error × dt
   *   5. Determine whether to apply the tentative integral (conditional integration):
   *        - rawOutput_tentative = pTerm + tentativeIntegral + dTerm
   *        - isSaturatedHigh = rawOutput_tentative > outputMax
   *        - isSaturatedLow  = rawOutput_tentative < outputMin
   *        - Block integration if (saturatedHigh AND error > 0) — integrating further
   *          pushes deeper into positive saturation.
   *        - Block integration if (saturatedLow AND error < 0) — same logic, negative.
   *        - Otherwise allow: also allows integration when saturated but error has
   *          reversed (helping unsaturate).
   *   6. Compute finalRawOutput = pTerm + (protected)integralTerm + dTerm
   *   7. Store finalRawOutput as lastRawOutput (for telemetry)
   *   8. Clamp to [outputMin, outputMax] and return.
   *
   * @param setpoint     Target body angle [rad].
   * @param measurement  Estimated body angle from the sensor/estimator [rad].
   *                     MUST be estimator output — never ground truth (R2.1).
   * @param dt           Elapsed time since last update [s].
   * @returns            Control output [N·m], clamped to [outputMin, outputMax].
   */
  update(setpoint: number, measurement: number, dt: number): number {
    if (dt <= 0) return 0; // Guard against zero/negative dt

    const { kp, ki, kd, outputMin, outputMax } = this.params;

    // Step 1: error
    const error = setpoint - measurement;

    // Step 2: proportional term
    const pTerm = kp * error;

    // Step 3: derivative-on-measurement (zero on first call — no kick at startup)
    let dMeasurement = 0;
    if (this.hasLastMeasurement) {
      dMeasurement = (measurement - this.lastMeasurement) / dt;
    } else {
      this.hasLastMeasurement = true;
    }
    this.lastMeasurement = measurement;
    // dError/dt = −dMeasurement/dt for constant setpoint → dTerm = −kd × dMeasurement
    const dTerm = -kd * dMeasurement;

    // Step 4: tentative integral (not yet guarded)
    const tentativeIntegral = this.integralTerm + ki * error * dt;

    // Step 5: conditional integration — windup protection
    // Compute the tentative raw output to detect saturation direction.
    const rawTentative = pTerm + tentativeIntegral + dTerm;
    const isSaturatedHigh = rawTentative > outputMax;
    const isSaturatedLow  = rawTentative < outputMin;
    // Block integral update only when it would push deeper into saturation.
    // Allow update if: not saturated at all, OR error is pulling OUT of saturation.
    const blockIntegral =
      (isSaturatedHigh && error > 0) ||
      (isSaturatedLow  && error < 0);

    if (!blockIntegral) {
      this.integralTerm = tentativeIntegral;
    }
    // else: hold integralTerm unchanged (do not wind up)

    // Step 6: final raw output with protected integral
    const finalRawOutput = pTerm + this.integralTerm + dTerm;

    // Step 7: store for telemetry — must be AFTER windup protection (not before)
    this.lastRawOutput = finalRawOutput;

    // Step 8: clamp and return
    if (finalRawOutput > outputMax) return outputMax;
    if (finalRawOutput < outputMin) return outputMin;
    return finalRawOutput;
  }

  /**
   * Live-updates PID gains without resetting integral or derivative state.
   * Applied on the next call to update(). Supports live gain tuning from the UI.
   */
  setGains(kp: number, ki: number, kd: number): void {
    this.params.kp = kp;
    this.params.ki = ki;
    this.params.kd = kd;
  }

  /**
   * Resets the controller's internal state (integral accumulator, derivative memory).
   * Call on simulation reset — does NOT reset gains or output limits.
   */
  reset(): void {
    this.integralTerm = 0;
    this.hasLastMeasurement = false;
    this.lastMeasurement = 0;
    this.lastRawOutput = 0;
  }

  /**
   * Returns the last computed raw output BEFORE clamping [N·m].
   * This is the value after windup protection is applied but before the actuator
   * limit clamps it. Positive values > outputMax indicate saturation. Used by
   * telemetry to visualize controller saturation events in Phase 5.
   */
  getLastRawOutput(): number {
    return this.lastRawOutput;
  }

  /**
   * Returns the current integral accumulator value [N·m·s].
   * Exposed for telemetry and debug — useful for diagnosing windup behavior.
   */
  getIntegralTerm(): number {
    return this.integralTerm;
  }
}
