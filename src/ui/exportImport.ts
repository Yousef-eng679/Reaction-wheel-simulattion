/**
 * exportImport.ts
 *
 * Data export and import — CSV telemetry export + JSON config round-trip.
 *
 * Functions (PRD §6.8):
 *   - Export full telemetry log as downloadable CSV.
 *   - Export current parameter set as downloadable JSON.
 *   - Import a JSON config file to restore an exact scenario.
 *
 * R2.3 — Reads telemetry from SimulationLoop's public API only.
 *         No direct access to core physics/control internals.
 * R4.3 — No backend, no server. Uses browser download APIs (Blob + URL.createObjectURL).
 *
 * Phase 0: function signatures only. Bodies throw 'not implemented'.
 * Phase 6 will implement the full export/import logic.
 */

import type { SimSnapshot } from '../sim/simulationLoop.ts';
import type { SimulationConfig } from '../sim/simulationLoop.ts';

/**
 * CSV column header names for telemetry export.
 * Keep in sync with the fields written by exportTelemetryCSV.
 */
export const CSV_COLUMNS = [
  'time_s',
  'true_angle_rad',
  'true_omega2_rad_s',
  'wheel_omega1_rad_s',
  'estimated_angle_rad',
  'control_output_nm',
  'raw_control_output_nm',
  'error_rad',
  'gyro_reading_rad_s',
  'accel_angle_rad',
] as const;

/**
 * Exports the given telemetry snapshots as a CSV file and triggers a browser download.
 *
 * @param telemetry  Array of SimSnapshots to export. Typically the full ring buffer.
 * @param filename   Suggested download filename (default: 'reaction_wheel_telemetry.csv').
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 6.
 */
export function exportTelemetryCSV(
  _telemetry: SimSnapshot[],
  _filename?: string,
): void {
  throw new Error('not implemented');
}

/**
 * Exports the current simulation configuration as a JSON file and triggers a browser download.
 * The exported JSON can be re-imported via importConfigJSON to reproduce the exact scenario.
 *
 * @param config    Current SimulationConfig to serialize.
 * @param filename  Suggested download filename (default: 'reaction_wheel_config.json').
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 6.
 */
export function exportConfigJSON(
  _config: SimulationConfig,
  _filename?: string,
): void {
  throw new Error('not implemented');
}

/**
 * Parses and validates a JSON config file (from a file input element) and returns
 * a SimulationConfig that can be used to reconstruct the simulation exactly.
 *
 * @param file  File object from an <input type="file"> element.
 * @returns     Promise resolving to the parsed SimulationConfig.
 * @throws      If the file is invalid JSON or fails validation.
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 6.
 */
export async function importConfigJSON(
  _file: File,
): Promise<SimulationConfig> {
  throw new Error('not implemented');
}
