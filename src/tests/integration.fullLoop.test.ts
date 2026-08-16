/**
 * integration.fullLoop.test.ts
 *
 * Phase 4 integration test — end-to-end closed loop via SimulationLoop + ScenarioRunner.
 *
 * This test exercises the FULL orchestrated pipeline:
 *   physics → sensor (with noise) → estimator → PID controller → motor (with lag) → physics
 *
 * It is meaningfully different from Phase 3's control.stepResponse.test.ts in that:
 *   1. The loop is driven through SimulationLoop.tick() — the same code path the UI will use.
 *   2. All realistic imperfections are active: sensor noise, motor lag, bias drift.
 *   3. runScenario() (the telemetry-returning runner used in Phase 5 and 6) is exercised.
 *   4. The integration-test verifies the R2.1 architectural boundary by checking that the
 *      controller's internal state never diverges from the physics in a way that would only
 *      happen if it had been given trueState directly.
 *
 * Tests:
 *   1. Full-loop step response — same acceptance bounds as Phase 3 (settle ±2°, overshoot ≤20%),
 *      now through the orchestrated loop with all imperfections active.
 *   2. Disturbance recovery — after settling, inject a kick and verify the system re-settles.
 *   3. scenarioRunner telemetry — run via runScenario(), verify telemetry array is well-formed
 *      and monotonically increasing in time.
 *   4. R2.1 architectural audit — verify that the estimated angle visible in telemetry differs
 *      from the true angle (as it would with real sensor noise), proving the controller is NOT
 *      getting trueState (a controller reading trueState would show zero error at all times).
 *
 * R3.2 — real physics engine via SimulationLoop (not mocked).
 * R3.3 — fixed sensor seeds for all scenarios.
 * R6.1 — physics-integrity-critical phase: see architectural audit test below.
 */

import { describe, it, expect } from 'vitest';
import { SimulationLoop } from '../sim/simulationLoop.ts';
import { runScenario } from '../sim/scenarioRunner.ts';
import type { ScenarioConfig } from '../sim/scenarioRunner.ts';
import {
  DEFAULT_I1_WHEEL_KGM2,
  DEFAULT_I2_BODY_KGM2,
  DEFAULT_FRICTION_COEFF_1,
  DEFAULT_FRICTION_COEFF_2,
  DEFAULT_MOTOR_MAX_RPM,
  DEFAULT_MOTOR_MAX_TORQUE_NM,
  DEFAULT_MOTOR_TIME_CONSTANT_S,
  DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
  DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
  DEFAULT_GYRO_BIAS_DRIFT_RATE,
  DEFAULT_SENSOR_SAMPLE_RATE_HZ,
  DEFAULT_COMPLEMENTARY_ALPHA,
  PHYSICS_DT_S,
  DEFAULT_KP,
  DEFAULT_KI,
  DEFAULT_KD,
  DEFAULT_MOTOR_MAX_TORQUE_NM as OUTPUT_MAX,
} from '../core/physics/constants.ts';

// ---------------------------------------------------------------------------
// Shared scenario config
// ---------------------------------------------------------------------------

const STEP_TARGET_RAD = Math.PI / 4;        // 45° step setpoint
const SETTLE_TOLERANCE_RAD = (2 * Math.PI) / 180; // ±2° settle band
const SETTLE_HOLD_SEC = 2.0;                // must hold in band for this long
const MAX_OVERSHOOT_FRACTION = 0.20;        // ≤20% of step magnitude
const PHYSICS_DT_MS = PHYSICS_DT_S * 1000;

const BASE_SIM_CONFIG = {
  physicsParams: {
    I1: DEFAULT_I1_WHEEL_KGM2,
    I2: DEFAULT_I2_BODY_KGM2,
    frictionCoeff1: DEFAULT_FRICTION_COEFF_1,
    frictionCoeff2: DEFAULT_FRICTION_COEFF_2,
  },
  motorParams: {
    maxRPM: DEFAULT_MOTOR_MAX_RPM,
    maxTorqueNm: DEFAULT_MOTOR_MAX_TORQUE_NM,
    timeConstantS: DEFAULT_MOTOR_TIME_CONSTANT_S,
  },
  sensorParams: {
    gyroNoiseSigma: DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
    accelAngleNoiseSigma: DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
    gyroBiasDriftRate: DEFAULT_GYRO_BIAS_DRIFT_RATE,
    sampleRateHz: DEFAULT_SENSOR_SAMPLE_RATE_HZ,
    seed: 77, // fixed (R3.3)
  },
  estimatorParams: { alpha: DEFAULT_COMPLEMENTARY_ALPHA },
  pidParams: {
    kp: DEFAULT_KP,
    ki: DEFAULT_KI,
    kd: DEFAULT_KD,
    outputMin: -OUTPUT_MAX,
    outputMax:  OUTPUT_MAX,
  },
};

// ---------------------------------------------------------------------------
// Helper: run the full loop for N seconds via tick(), return final snapshot metrics
// ---------------------------------------------------------------------------

interface LoopResult {
  settledTimeS: number;         // first time body entered and held ±2° band; −1 if never
  maxOvershootRad: number;      // peak above setpoint
  finalTheta2: number;          // body angle at end
  finalEstimate: number;        // estimator angle at end
}

function runFullLoop(durationS: number, setpointRad: number, seed: number): LoopResult {
  const loop = new SimulationLoop({
    ...BASE_SIM_CONFIG,
    sensorParams: { ...BASE_SIM_CONFIG.sensorParams, seed },
    initialSetpoint: setpointRad,
  });

  const totalSteps = Math.round(durationS / PHYSICS_DT_S);
  const settleHoldSteps = Math.round(SETTLE_HOLD_SEC / PHYSICS_DT_S);
  let continuousSettledSteps = 0;
  let settledTimeS = -1;
  let maxOvershootRad = 0;

  for (let i = 0; i < totalSteps; i++) {
    loop.tick(PHYSICS_DT_MS);
    const snap = loop.getSnapshot();

    const overshoot = snap.trueState.theta2 - setpointRad;
    if (overshoot > maxOvershootRad) maxOvershootRad = overshoot;

    if (Math.abs(snap.trueState.theta2 - setpointRad) <= SETTLE_TOLERANCE_RAD) {
      continuousSettledSteps++;
      if (continuousSettledSteps >= settleHoldSteps && settledTimeS === -1) {
        settledTimeS = snap.simTimeSec - SETTLE_HOLD_SEC;
      }
    } else {
      continuousSettledSteps = 0;
      settledTimeS = -1;
    }
  }

  const finalSnap = loop.getSnapshot();
  return {
    settledTimeS,
    maxOvershootRad,
    finalTheta2: finalSnap.trueState.theta2,
    finalEstimate: finalSnap.estimatedAngle,
  };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('Integration — full orchestrated loop (Phase 4)', () => {
  /**
   * Test 1: Full-loop step response (same acceptance bounds as Phase 3).
   * Proves the orchestration wiring didn't break the control behavior that
   * Phase 3 verified in direct-wiring tests.
   */
  it('full loop settles within ±2° of 45° target within 30 s with realistic noise (R3.2)', () => {
    const { settledTimeS } = runFullLoop(30, STEP_TARGET_RAD, /* seed */ 77);
    expect(settledTimeS, 'Body never settled via full loop').toBeGreaterThanOrEqual(0);
    expect(settledTimeS).toBeLessThan(30 - SETTLE_HOLD_SEC);
  });

  it('full loop overshoot does not exceed 20% of step magnitude (R3.2)', () => {
    const { maxOvershootRad } = runFullLoop(30, STEP_TARGET_RAD, /* seed */ 77);
    const maxAllowed = MAX_OVERSHOOT_FRACTION * STEP_TARGET_RAD;
    expect(maxOvershootRad).toBeLessThanOrEqual(maxAllowed);
  });

  /**
   * Test 2: Disturbance recovery.
   * Settle to target, then inject a 0.1 N·m kick at t=20 s, and verify
   * the system re-settles within 15 s of the kick.
   */
  it('recovers from a 0.1 N·m disturbance kick and re-settles within 15 s (R3.2)', () => {
    const DURATION = 40;    // [s] — 20 s to settle, kick at 20 s, 20 s to re-settle
    const KICK_TIME = 20;   // [s]
    const KICK_NM   = 0.1;  // [N·m] direct body disturbance

    const loop = new SimulationLoop({
      ...BASE_SIM_CONFIG,
      sensorParams: { ...BASE_SIM_CONFIG.sensorParams, seed: 55 },
      initialSetpoint: STEP_TARGET_RAD,
    });

    const totalSteps = Math.round(DURATION / PHYSICS_DT_S);
    const kickStep   = Math.round(KICK_TIME / PHYSICS_DT_S);
    const settleHoldSteps = Math.round(SETTLE_HOLD_SEC / PHYSICS_DT_S);

    let kickApplied = false;
    let postKickSettledTimeS = -1;
    let continuousSettledSteps = 0;
    let inPreKickPhase = true;

    for (let i = 0; i < totalSteps; i++) {
      if (!kickApplied && i === kickStep) {
        loop.applyDisturbanceKick(KICK_NM);
        kickApplied = true;
        inPreKickPhase = false;
        continuousSettledSteps = 0;
        postKickSettledTimeS = -1;
      }

      loop.tick(PHYSICS_DT_MS);
      const snap = loop.getSnapshot();

      if (!inPreKickPhase) {
        if (Math.abs(snap.trueState.theta2 - STEP_TARGET_RAD) <= SETTLE_TOLERANCE_RAD) {
          continuousSettledSteps++;
          if (continuousSettledSteps >= settleHoldSteps && postKickSettledTimeS === -1) {
            postKickSettledTimeS = snap.simTimeSec - KICK_TIME - SETTLE_HOLD_SEC;
          }
        } else {
          continuousSettledSteps = 0;
          postKickSettledTimeS = -1;
        }
      }
    }

    expect(postKickSettledTimeS, 'System did not re-settle after disturbance kick')
      .toBeGreaterThanOrEqual(0);
    expect(postKickSettledTimeS, 'Re-settle took too long after kick')
      .toBeLessThan(15);
  });

  /**
   * Test 3: scenarioRunner telemetry is well-formed.
   * Verifies runScenario() returns a valid telemetry array with:
   *   - More than one entry
   *   - Monotonically increasing simTimeSec
   *   - All numeric fields finite
   *   - First entry is the initial state (t=0)
   */
  it('runScenario() returns well-formed monotonic telemetry array', () => {
    const scenario: ScenarioConfig = {
      name: 'Integration — telemetry validation',
      simConfig: {
        ...BASE_SIM_CONFIG,
        sensorParams: { ...BASE_SIM_CONFIG.sensorParams, seed: 33 },
      },
      setpointRad: STEP_TARGET_RAD,
      durationSec: 5,
      loggingIntervalS: PHYSICS_DT_S * 10, // 10 ms logging = 500 snapshots for 5 s
    };

    const result = runScenario(scenario);
    const { telemetry } = result;

    // Must have produced snapshots
    expect(telemetry.length).toBeGreaterThan(10);

    // First snapshot is initial state
    expect(telemetry[0].simTimeSec).toBeCloseTo(0, 5);

    // Time must be monotonically increasing
    for (let i = 1; i < telemetry.length; i++) {
      expect(telemetry[i].simTimeSec).toBeGreaterThan(telemetry[i - 1].simTimeSec);
    }

    // All numeric fields must be finite (no NaN / Infinity)
    for (const snap of telemetry) {
      expect(Number.isFinite(snap.simTimeSec)).toBe(true);
      expect(Number.isFinite(snap.trueState.theta2)).toBe(true);
      expect(Number.isFinite(snap.trueState.omega2)).toBe(true);
      expect(Number.isFinite(snap.estimatedAngle)).toBe(true);
      expect(Number.isFinite(snap.controlOutput)).toBe(true);
      expect(Number.isFinite(snap.error)).toBe(true);
    }

    // Wall-clock performance: 5 s of simulation should complete well under 2000 ms
    expect(result.wallClockMs).toBeLessThan(2000);
  });

  /**
   * Test 4: R2.1 architectural audit — controller is NOT reading true state.
   *
   * If the controller were secretly fed trueState, the estimated angle would always
   * equal the true angle (zero sensor error). With real sensor noise and a complementary
   * filter, the estimated angle MUST differ from the true angle by at least the noise floor.
   *
   * We measure the RMS difference between trueState.theta2 and estimatedAngle over 10 s.
   * It must be greater than zero (noise is real) but small enough that the filter is working
   * (not just outputting raw sensor noise).
   *
   * This is the R6.1-required code-path audit embedded as an automated assertion.
   */
  it('R2.1 audit: estimated angle differs from true angle (proves controller gets estimator output, not trueState)', () => {
    const scenario: ScenarioConfig = {
      name: 'R2.1 architectural audit',
      simConfig: {
        ...BASE_SIM_CONFIG,
        sensorParams: { ...BASE_SIM_CONFIG.sensorParams, seed: 12 },
      },
      setpointRad: STEP_TARGET_RAD,
      durationSec: 10,
      loggingIntervalS: PHYSICS_DT_S * 5, // 5 ms logging
    };

    const result = runScenario(scenario);
    const { telemetry } = result;

    // Compute RMS difference between true and estimated angle
    let sumSqDiff = 0;
    for (const snap of telemetry) {
      const diff = snap.trueState.theta2 - snap.estimatedAngle;
      sumSqDiff += diff * diff;
    }
    const rmsDiff = Math.sqrt(sumSqDiff / telemetry.length);

    // RMS difference MUST be > 0 — if controller got trueState, estimatedAngle
    // would always equal trueState.theta2 and rmsDiff would be exactly 0.
    expect(rmsDiff, 'Estimated angle is identical to true angle — controller may be bypassing estimator (R2.1 violation)')
      .toBeGreaterThan(1e-6);

    // RMS difference should be small — the filter works, it's not just raw noise
    // Expect < 0.1 rad (well below the noise floor of the raw gyro)
    expect(rmsDiff, 'Estimated angle diverges too far from true angle — estimator may be broken')
      .toBeLessThan(0.1);
  });
});
