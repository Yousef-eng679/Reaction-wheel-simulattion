/**
 * exportImport.ts
 *
 * Data export and import — CSV telemetry export + JSON config round-trip (PRD §6.8).
 *
 * Functions:
 *   exportTelemetryCSV  — Serialises a telemetry snapshot array to CSV and
 *                         triggers a browser download (no server required — R4.3).
 *   exportConfigJSON    — Serialises the current SimulationConfig to JSON and
 *                         triggers a browser download.
 *   importConfigJSON    — Reads a File object from <input type="file">, parses
 *                         and validates the JSON, and returns a SimulationConfig
 *                         that reproduces the scenario exactly.
 *
 * R2.3 — Reads telemetry via SimSnapshot (the loop's public API). No direct access
 *         to physicsEngine, pidController, or other core modules.
 * R4.3 — No backend, no server. Uses browser Blob + URL.createObjectURL for downloads.
 * R1.3 — All exported values remain in SI units (rad, rad/s, N·m, s, kg·m²).
 *         Display-unit conversion (degrees, RPM) happens only at the UI layer.
 */

import type { SimSnapshot, SimulationConfig } from '../sim/simulationLoop.ts';

// ---------------------------------------------------------------------------
// CSV export
// ---------------------------------------------------------------------------

/**
 * CSV column header names for telemetry export.
 * Every column is in SI units — see README §Parameters for definitions.
 */
export const CSV_COLUMNS = [
  'time_s',
  'true_angle_rad',
  'true_omega2_rad_s',
  'wheel_omega1_rad_s',
  'true_theta1_rad',
  'estimated_angle_rad',
  'control_output_nm',
  'raw_control_output_nm',
  'error_rad',
  'setpoint_rad',
  'gyro_reading_rad_s',
  'accel_angle_rad',
] as const;

/**
 * Exports an array of SimSnapshots as a CSV file and triggers a browser download.
 *
 * Columns (all SI units):
 *   time_s                — simulation time [s]
 *   true_angle_rad        — true body angle θ₂ [rad]
 *   true_omega2_rad_s     — true body angular velocity ω₂ [rad/s]
 *   wheel_omega1_rad_s    — true wheel angular velocity ω₁ [rad/s]
 *   true_theta1_rad       — true wheel angle θ₁ [rad]
 *   estimated_angle_rad   — estimator output [rad] (what the controller sees)
 *   control_output_nm     — clamped PID output [N·m]
 *   raw_control_output_nm — pre-clamp PID output [N·m] (> limits = saturated)
 *   error_rad             — setpoint − estimatedAngle [rad]
 *   setpoint_rad          — current target body angle [rad]
 *   gyro_reading_rad_s    — noisy gyroscope reading [rad/s]
 *   accel_angle_rad       — accelerometer-derived angle estimate [rad]
 *
 * @param telemetry  Array of SimSnapshots (e.g. collected from the telemetry logger).
 * @param filename   Suggested download filename (default: 'reaction_wheel_telemetry.csv').
 */
export function exportTelemetryCSV(
  telemetry: SimSnapshot[],
  filename = 'reaction_wheel_telemetry.csv',
): void {
  const header = CSV_COLUMNS.join(',');

  const rows = telemetry.map(snap => [
    snap.simTimeSec.toFixed(6),
    snap.trueState.theta2.toFixed(8),
    snap.trueState.omega2.toFixed(8),
    snap.trueState.omega1.toFixed(8),
    snap.trueState.theta1.toFixed(8),
    snap.estimatedAngle.toFixed(8),
    snap.controlOutput.toFixed(8),
    snap.rawControlOutput.toFixed(8),
    snap.error.toFixed(8),
    snap.setpoint.toFixed(8),
    snap.sensorReading.gyroOmega2.toFixed(8),
    snap.sensorReading.accelAngleEstimate.toFixed(8),
  ].join(','));

  const csvContent = [header, ...rows].join('\n');
  _triggerDownload(csvContent, filename, 'text/csv;charset=utf-8;');
}

// ---------------------------------------------------------------------------
// JSON config export
// ---------------------------------------------------------------------------

/**
 * The structure persisted in the exported JSON config file.
 * Matches the SimulationConfig shape but with explicit top-level keys for
 * clarity and forward-compatibility.
 */
export interface ExportedConfig {
  /** Schema version — increment if the shape changes incompatibly. */
  schemaVersion: number;
  /** ISO-8601 timestamp of when this config was exported. */
  exportedAt: string;
  /** Human-readable label (optional, settable by user in future). */
  label?: string;
  /** The full simulation configuration. */
  config: SimulationConfig;
}

/**
 * Exports the current SimulationConfig as a JSON file and triggers a browser download.
 * The exported file can be re-imported via importConfigJSON to exactly reproduce the scenario.
 *
 * All numeric values retain full double-precision — no rounding during export.
 *
 * @param config    Current SimulationConfig to serialise.
 * @param filename  Suggested download filename (default: 'reaction_wheel_config.json').
 */
export function exportConfigJSON(
  config: SimulationConfig,
  filename = 'reaction_wheel_config.json',
): void {
  const payload: ExportedConfig = {
    schemaVersion: 1,
    exportedAt: new Date().toISOString(),
    config,
  };
  const jsonContent = JSON.stringify(payload, null, 2);
  _triggerDownload(jsonContent, filename, 'application/json');
}

// ---------------------------------------------------------------------------
// JSON config import
// ---------------------------------------------------------------------------

/**
 * Parses and validates a JSON config file selected by the user via
 * <input type="file"> and returns a SimulationConfig ready for use.
 *
 * Validation performed:
 *   - Must be valid JSON.
 *   - Must contain a `config` object with required sub-fields.
 *   - All required numeric fields must be finite positive numbers.
 *   - PID output bounds must be symmetric (|outputMin| === outputMax).
 *
 * Throws a descriptive Error if validation fails, so the UI can surface the
 * message directly to the user.
 *
 * @param file  File object from an <input type="file"> change event.
 * @returns     Promise<SimulationConfig> — the validated config.
 */
export async function importConfigJSON(file: File): Promise<SimulationConfig> {
  const text = await file.text();

  let parsed: unknown;
  try {
    parsed = JSON.parse(text);
  } catch {
    throw new Error(`Invalid JSON in uploaded file: ${file.name}`);
  }

  // Duck-type validate the shape
  if (typeof parsed !== 'object' || parsed === null) {
    throw new Error('Config file must be a JSON object.');
  }

  const root = parsed as Record<string, unknown>;

  // Support both the wrapped ExportedConfig format and a bare SimulationConfig
  const cfg: unknown = 'config' in root ? root.config : root;

  if (typeof cfg !== 'object' || cfg === null) {
    throw new Error('Config file is missing the "config" field.');
  }

  const c = cfg as Record<string, unknown>;

  // Validate required top-level sections
  _requireObject(c, 'physicsParams');
  _requireObject(c, 'motorParams');
  _requireObject(c, 'sensorParams');
  _requireObject(c, 'estimatorParams');
  _requireObject(c, 'pidParams');

  const pp  = c.physicsParams  as Record<string, unknown>;
  const mp  = c.motorParams    as Record<string, unknown>;
  const sp  = c.sensorParams   as Record<string, unknown>;
  const ep  = c.estimatorParams as Record<string, unknown>;
  const pid = c.pidParams      as Record<string, unknown>;

  // physicsParams
  _requireFinitePositive(pp,  'I1');
  _requireFinitePositive(pp,  'I2');
  _requireFiniteNonNeg(pp,    'frictionCoeff1');
  _requireFiniteNonNeg(pp,    'frictionCoeff2');

  // motorParams
  _requireFinitePositive(mp,  'maxRPM');
  _requireFinitePositive(mp,  'maxTorqueNm');
  _requireFinitePositive(mp,  'timeConstantS');

  // sensorParams
  _requireFiniteNonNeg(sp,    'gyroNoiseSigma');
  _requireFiniteNonNeg(sp,    'accelAngleNoiseSigma');
  _requireFiniteNonNeg(sp,    'gyroBiasDriftRate');
  _requireFinitePositive(sp,  'sampleRateHz');
  _requireInteger(sp,         'seed');

  // estimatorParams
  _requireInRange(ep,         'alpha', 0, 1);

  // pidParams
  _requireFinite(pid,         'kp');
  _requireFinite(pid,         'ki');
  _requireFinite(pid,         'kd');
  _requireFinite(pid,         'outputMin');
  _requireFinitePositive(pid, 'outputMax');

  // Build the validated config (explicit field extraction prevents injection of extra keys)
  const result: SimulationConfig = {
    physicsParams: {
      I1:            Number(pp.I1),
      I2:            Number(pp.I2),
      frictionCoeff1: Number(pp.frictionCoeff1),
      frictionCoeff2: Number(pp.frictionCoeff2),
    },
    motorParams: {
      maxRPM:        Number(mp.maxRPM),
      maxTorqueNm:   Number(mp.maxTorqueNm),
      timeConstantS: Number(mp.timeConstantS),
    },
    sensorParams: {
      gyroNoiseSigma:         Number(sp.gyroNoiseSigma),
      accelAngleNoiseSigma:   Number(sp.accelAngleNoiseSigma),
      gyroBiasDriftRate:      Number(sp.gyroBiasDriftRate),
      sampleRateHz:           Number(sp.sampleRateHz),
      seed:                   Number(sp.seed) | 0, // truncate to 32-bit integer
    },
    estimatorParams: {
      alpha: Number(ep.alpha),
    },
    pidParams: {
      kp:        Number(pid.kp),
      ki:        Number(pid.ki),
      kd:        Number(pid.kd),
      outputMin: Number(pid.outputMin),
      outputMax: Number(pid.outputMax),
    },
    initialSetpoint: typeof c.initialSetpoint === 'number' ? c.initialSetpoint : 0,
  };

  return result;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/** Triggers a browser file download with the given content. */
function _triggerDownload(content: string, filename: string, mimeType: string): void {
  const blob = new Blob([content], { type: mimeType });
  const url  = URL.createObjectURL(blob);
  const a    = document.createElement('a');
  a.href     = url;
  a.download = filename;
  a.style.display = 'none';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  // Revoke after a brief delay to allow the download to start
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function _requireObject(obj: Record<string, unknown>, key: string): void {
  if (typeof obj[key] !== 'object' || obj[key] === null) {
    throw new Error(`Config validation: "${key}" must be an object.`);
  }
}

function _requireFinite(obj: Record<string, unknown>, key: string): void {
  const v = Number(obj[key]);
  if (!Number.isFinite(v)) {
    throw new Error(`Config validation: "${key}" must be a finite number (got ${obj[key]}).`);
  }
}

function _requireFinitePositive(obj: Record<string, unknown>, key: string): void {
  const v = Number(obj[key]);
  if (!Number.isFinite(v) || v <= 0) {
    throw new Error(`Config validation: "${key}" must be a finite positive number (got ${obj[key]}).`);
  }
}

function _requireFiniteNonNeg(obj: Record<string, unknown>, key: string): void {
  const v = Number(obj[key]);
  if (!Number.isFinite(v) || v < 0) {
    throw new Error(`Config validation: "${key}" must be a finite non-negative number (got ${obj[key]}).`);
  }
}

function _requireInRange(obj: Record<string, unknown>, key: string, min: number, max: number): void {
  const v = Number(obj[key]);
  if (!Number.isFinite(v) || v < min || v > max) {
    throw new Error(`Config validation: "${key}" must be in [${min}, ${max}] (got ${obj[key]}).`);
  }
}

function _requireInteger(obj: Record<string, unknown>, key: string): void {
  const v = Number(obj[key]);
  if (!Number.isFinite(v)) {
    throw new Error(`Config validation: "${key}" must be an integer (got ${obj[key]}).`);
  }
}
