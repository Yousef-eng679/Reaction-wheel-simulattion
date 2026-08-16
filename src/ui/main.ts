/**
 * src/ui/main.ts — Application entry point and RAF loop (Phase 5 + 6).
 *
 * Responsibilities:
 *   1. Build the DOM layout (header readouts, canvas, chart canvases, control panel).
 *   2. Construct a SimulationLoop with default parameters.
 *   3. Wire ControlPanel callbacks → SimulationLoop public API (R2.3).
 *   4. Start the requestAnimationFrame render loop.
 *   5. Each RAF frame:
 *        a. Advance physics via loop.tick(realDtMs)  [accumulator, not render-locked — R2.5]
 *        b. Read snapshot via loop.getSnapshot()
 *        c. Log to telemetry charts (at a sub-sampled rate to avoid overwhelming the buffer)
 *        d. Render canvas and charts
 *        e. Update panel readouts
 *
 * Critical:
 *   - RAF delta (realDtMs) is passed to SimulationLoop.tick() which accumulates it
 *     internally and fires fixed PHYSICS_DT_S steps — NEVER used as a physics dt directly.
 *   - Physics can step many times per frame if the frame is slow (catch-up), or zero times
 *     if very fast (no-op). The accumulator handles all of this.
 *
 * R2.3 — This file ONLY imports SimulationLoop (orchestrator), CanvasRenderer,
 *         TelemetryChart, TelemetryLogger, and ControlPanel. It does NOT import
 *         physicsEngine, pidController, motorModel, sensorModel, or estimator directly.
 *
 * Live parameter changes:
 *   Parameters that would require reconstructing the SimulationLoop (e.g. sensor seed,
 *   initial conditions) trigger a reset + rebuild. Parameters that the loop exposes as
 *   live setters (setpoint, gains) are applied immediately via the loop's API.
 *   For parameters not yet exposed as live setters (I1, I2, friction, motor params,
 *   sensor noise, alpha) — the loop is rebuilt and reset. This is acceptable because
 *   these are physical parameters you would need to reset a real system to change anyway.
 */

import { SimulationLoop } from '../sim/simulationLoop.ts';
import type { SimulationConfig } from '../sim/simulationLoop.ts';
import { CanvasRenderer } from './canvasRenderer.ts';
import { TelemetryChart, TelemetryLogger } from './telemetryChart.ts';
import { ControlPanel } from './controlPanel.ts';
import { exportTelemetryCSV, exportConfigJSON, importConfigJSON } from './exportImport.ts';
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
  DEFAULT_KP,
  DEFAULT_KI,
  DEFAULT_KD,
  RAD_TO_DEG,
  RAD_S_TO_RPM,
} from '../core/physics/constants.ts';

// ---------------------------------------------------------------------------
// Application state (mutable, lives at module scope)
// ---------------------------------------------------------------------------

let simConfig: SimulationConfig = buildDefaultConfig();
let loop: SimulationLoop = new SimulationLoop(simConfig);
let paused = false;
let lastFrameTimeMs: number | null = null;

// Telemetry sub-sampling: log one record every N RAF frames
const LOG_EVERY_MS = 20; // log ~50 Hz
let logAccumulatorMs = 0;

// Chart window: 10 seconds of data at 50 Hz = 500 samples
const CHART_WINDOW = 500;

// Telemetry snapshot buffer for CSV export (PRD §6.8)
// Keeps the last MAX_EXPORT_SNAPSHOTS logged snapshots for on-demand CSV download.
const MAX_EXPORT_SNAPSHOTS = 30_000; // ~10 min at 50 Hz
const exportBuffer: ReturnType<typeof loop.getSnapshot>[] = [];

// ---------------------------------------------------------------------------
// Build default SimulationConfig from constants
// ---------------------------------------------------------------------------

function buildDefaultConfig(overrides: Partial<SimulationConfig> = {}): SimulationConfig {
  return {
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
      seed: Date.now() & 0xffffffff, // random seed for live UI (non-reproducible)
    },
    estimatorParams: { alpha: DEFAULT_COMPLEMENTARY_ALPHA },
    pidParams: {
      kp: DEFAULT_KP,
      ki: DEFAULT_KI,
      kd: DEFAULT_KD,
      outputMin: -DEFAULT_MOTOR_MAX_TORQUE_NM,
      outputMax:  DEFAULT_MOTOR_MAX_TORQUE_NM,
    },
    initialSetpoint: 0,
    ...overrides,
  };
}

// ---------------------------------------------------------------------------
// DOM construction
// ---------------------------------------------------------------------------

const appEl = document.querySelector<HTMLDivElement>('#app')!;
appEl.innerHTML = `
  <div id="header">
    <h1>⚙️ <span>Reaction Wheel</span> Attitude Control Simulator</h1>
    <div class="header-readouts">
      <div class="readout">
        <span class="readout-label">True Angle</span>
        <span class="readout-value true"  id="hdr-true">0.00°</span>
      </div>
      <div class="readout">
        <span class="readout-label">Estimated</span>
        <span class="readout-value est"   id="hdr-est">0.00°</span>
      </div>
      <div class="readout">
        <span class="readout-label">Body ω₂</span>
        <span class="readout-value omega" id="hdr-omega">0.000</span>
      </div>
      <div class="readout">
        <span class="readout-label">Wheel RPM</span>
        <span class="readout-value rpm"   id="hdr-rpm">0</span>
      </div>
    </div>
    <div class="toolbar">
      <button id="btn-kick-hdr" class="warn" title="Apply 0.5 N·m disturbance kick">💥 Kick</button>
      <button id="btn-export-csv" title="Download telemetry as CSV">⬇ CSV</button>
      <button id="btn-export-json" title="Download current config as JSON">⬇ Config</button>
      <label id="btn-import-json" title="Import a config JSON to restore a scenario" style="cursor:pointer">
        ⬆ Import
        <input type="file" id="file-import-input" accept=".json" style="display:none" />
      </label>
    </div>
  </div>

  <div id="panel"></div>

  <div id="viz">
    <canvas id="canvas-main"></canvas>
    <div id="sat-badge" class="sat-badge">⚡ SATURATED</div>
  </div>

  <div id="status">
    <div class="status-item"><span>Physics</span><span id="st-rate">1000 Hz</span></div>
    <div class="status-item"><span>Sensor</span><span id="st-sensor">200 Hz</span></div>
    <div class="status-item"><span>Sim time</span><span id="st-time">0.00 s</span></div>
    <div class="status-item"><span>FPS</span><span id="st-fps">—</span></div>
    <div class="status-item"><span>Control out</span><span id="st-ctrl">0.000 N·m</span></div>
  </div>

  <div id="charts">
    <div class="section-title" style="padding-bottom:0.2rem">Telemetry</div>
    <div class="legend">
      <div class="legend-item"><div class="legend-dot" style="background:var(--true-color)"></div>True θ</div>
      <div class="legend-item"><div class="legend-dot" style="background:var(--est-color)"></div>Est. θ</div>
    </div>
    <div class="chart-label">
      <span>Angle [deg]</span>
      <span class="chart-value" id="chart-val-angle">—</span>
    </div>
    <canvas id="chart-angle" class="chart-canvas" height="68"></canvas>

    <div class="chart-label">
      <span>ω₂ body [rad/s]</span>
      <span class="chart-value" id="chart-val-omega">—</span>
    </div>
    <canvas id="chart-omega" class="chart-canvas" height="52"></canvas>

    <div class="chart-label">
      <span>Wheel RPM</span>
      <span class="chart-value" id="chart-val-rpm">—</span>
    </div>
    <canvas id="chart-rpm" class="chart-canvas" height="52"></canvas>

    <div class="chart-label">
      <span>Control output [N·m]</span>
      <span class="chart-value" id="chart-val-ctrl">—</span>
    </div>
    <canvas id="chart-ctrl" class="chart-canvas" height="52"></canvas>

    <div class="chart-label">
      <span>Error [deg]</span>
      <span class="chart-value" id="chart-val-err">—</span>
    </div>
    <canvas id="chart-err" class="chart-canvas" height="52"></canvas>
  </div>
`;

// ---------------------------------------------------------------------------
// Canvas renderer
// ---------------------------------------------------------------------------

const mainCanvas = document.getElementById('canvas-main') as HTMLCanvasElement;
const vizEl = document.getElementById('viz') as HTMLDivElement;

function getVizSize(): { w: number; h: number } {
  return { w: vizEl.clientWidth, h: vizEl.clientHeight };
}

const { w: initW, h: initH } = getVizSize();
const renderer = new CanvasRenderer({ canvas: mainCanvas, width: initW || 400, height: initH || 400 });

// Resize canvas when the viz container resizes
const vizObserver = new ResizeObserver(() => {
  const { w, h } = getVizSize();
  if (w > 0 && h > 0) renderer.resize(w, h);
});
vizObserver.observe(vizEl);

// ---------------------------------------------------------------------------
// Telemetry charts — set canvas widths from container
// ---------------------------------------------------------------------------

function sizeChartCanvases(): void {
  const chartsEl = document.getElementById('charts')!;
  const cw = chartsEl.clientWidth - 16; // padding
  document.querySelectorAll<HTMLCanvasElement>('canvas.chart-canvas').forEach(c => {
    c.width = cw > 0 ? cw : 280;
    // height is set via HTML attribute
  });
}
sizeChartCanvases();
new ResizeObserver(sizeChartCanvases).observe(document.getElementById('charts')!);

function makeChart(canvasId: string, signals: { key: any; color: string; lineWidth?: number; dashed?: boolean }[], yLabel: string, yMin: number, yMax: number): TelemetryChart {
  const canvas = document.getElementById(canvasId) as HTMLCanvasElement;
  return new TelemetryChart({ canvas, signals, yLabel, yMin, yMax, windowSamples: CHART_WINDOW });
}

const chartAngle = makeChart('chart-angle', [
  { key: 'trueAngleDeg',      color: 'var(--true-color)', lineWidth: 2 },
  { key: 'estimatedAngleDeg', color: 'var(--est-color)',  lineWidth: 1.5, dashed: true },
], 'deg', 0, 0 /* auto */);

const chartOmega = makeChart('chart-omega', [
  { key: 'omega2RadS', color: 'var(--omega-color)', lineWidth: 1.5 },
], 'rad/s', 0, 0);

const chartRpm = makeChart('chart-rpm', [
  { key: 'wheelRpm', color: 'var(--rpm-color)', lineWidth: 1.5 },
], 'RPM', 0, 0);

const chartCtrl = makeChart('chart-ctrl', [
  { key: 'rawControlOutputNm', color: 'rgba(248,113,113,0.35)', lineWidth: 1, dashed: true },
  { key: 'controlOutputNm',    color: 'var(--ctrl-color)',       lineWidth: 1.5 },
], 'N·m', 0, 0);

const chartErr = makeChart('chart-err', [
  { key: 'errorDeg', color: 'var(--err-color)', lineWidth: 1.5 },
], 'deg', 0, 0);

const logger = new TelemetryLogger([chartAngle, chartOmega, chartRpm, chartCtrl, chartErr]);

// Header live value elements
const hdrTrue  = document.getElementById('hdr-true')!;
const hdrEst   = document.getElementById('hdr-est')!;
const hdrOmega = document.getElementById('hdr-omega')!;
const hdrRpm   = document.getElementById('hdr-rpm')!;
const stTime   = document.getElementById('st-time')!;
const stFps    = document.getElementById('st-fps')!;
const stCtrl   = document.getElementById('st-ctrl')!;
const satBadge = document.getElementById('sat-badge')!;

// ---------------------------------------------------------------------------
// Control panel
// ---------------------------------------------------------------------------

const panelEl = document.getElementById('panel')!;

// mutable live-param state (used when rebuilding the loop)
const liveParams = {
  i1: DEFAULT_I1_WHEEL_KGM2,
  i2: DEFAULT_I2_BODY_KGM2,
  friction1: DEFAULT_FRICTION_COEFF_1,
  friction2: DEFAULT_FRICTION_COEFF_2,
  maxRpm: DEFAULT_MOTOR_MAX_RPM,
  maxTorqueNm: DEFAULT_MOTOR_MAX_TORQUE_NM,
  motorTauS: DEFAULT_MOTOR_TIME_CONSTANT_S,
  gyroNoiseSigma: DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
  accelAngleNoiseSigma: DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
  alpha: DEFAULT_COMPLEMENTARY_ALPHA,
  kp: DEFAULT_KP,
  ki: DEFAULT_KI,
  kd: DEFAULT_KD,
  setpointRad: 0,
};

/** Rebuild the SimulationLoop with current liveParams (for physical params that can't hot-patch). */
function rebuildLoop(): void {
  simConfig = buildDefaultConfig({
    physicsParams: {
      I1: liveParams.i1,
      I2: liveParams.i2,
      frictionCoeff1: liveParams.friction1,
      frictionCoeff2: liveParams.friction2,
    },
    motorParams: {
      maxRPM: liveParams.maxRpm,
      maxTorqueNm: liveParams.maxTorqueNm,
      timeConstantS: liveParams.motorTauS,
    },
    sensorParams: {
      gyroNoiseSigma: liveParams.gyroNoiseSigma,
      accelAngleNoiseSigma: liveParams.accelAngleNoiseSigma,
      gyroBiasDriftRate: DEFAULT_GYRO_BIAS_DRIFT_RATE,
      sampleRateHz: DEFAULT_SENSOR_SAMPLE_RATE_HZ,
      seed: Date.now() & 0xffffffff,
    },
    estimatorParams: { alpha: liveParams.alpha },
    pidParams: {
      kp: liveParams.kp,
      ki: liveParams.ki,
      kd: liveParams.kd,
      outputMin: -liveParams.maxTorqueNm,
      outputMax:  liveParams.maxTorqueNm,
    },
    initialSetpoint: liveParams.setpointRad,
  });
  loop = new SimulationLoop(simConfig);
  logger.clearAll();
}

const panel = new ControlPanel({
  container: panelEl,
  setpointDeg: 0,
  kp: DEFAULT_KP, ki: DEFAULT_KI, kd: DEFAULT_KD,
  i1: DEFAULT_I1_WHEEL_KGM2, i2: DEFAULT_I2_BODY_KGM2,
  friction1: DEFAULT_FRICTION_COEFF_1, friction2: DEFAULT_FRICTION_COEFF_2,
  maxRpm: DEFAULT_MOTOR_MAX_RPM, maxTorqueNm: DEFAULT_MOTOR_MAX_TORQUE_NM,
  motorTauS: DEFAULT_MOTOR_TIME_CONSTANT_S,
  gyroNoiseSigma: DEFAULT_GYRO_NOISE_SIGMA_RAD_S,
  accelAngleNoiseSigma: DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD,
  alpha: DEFAULT_COMPLEMENTARY_ALPHA,
  callbacks: {
    // Live setters — applied immediately without rebuilding (R2.3: via SimulationLoop API)
    onSetpoint: (rad) => { liveParams.setpointRad = rad; loop.setSetpoint(rad); },
    onKp: (v) => { liveParams.kp = v; loop.setGains(liveParams.kp, liveParams.ki, liveParams.kd); },
    onKi: (v) => { liveParams.ki = v; loop.setGains(liveParams.kp, liveParams.ki, liveParams.kd); },
    onKd: (v) => { liveParams.kd = v; loop.setGains(liveParams.kp, liveParams.ki, liveParams.kd); },
    // Physical params — rebuild loop (state reset is appropriate for physical changes)
    onI1:         (v) => { liveParams.i1 = v;           rebuildLoop(); },
    onI2:         (v) => { liveParams.i2 = v;           rebuildLoop(); },
    onFriction1:  (v) => { liveParams.friction1 = v;    rebuildLoop(); },
    onFriction2:  (v) => { liveParams.friction2 = v;    rebuildLoop(); },
    onMaxRpm:     (v) => { liveParams.maxRpm = v;        rebuildLoop(); },
    onMaxTorque:  (v) => { liveParams.maxTorqueNm = v;  rebuildLoop(); },
    onMotorTau:   (v) => { liveParams.motorTauS = v;     rebuildLoop(); },
    onGyroNoise:  (v) => { liveParams.gyroNoiseSigma = v;         rebuildLoop(); },
    onAccelNoise: (v) => { liveParams.accelAngleNoiseSigma = v;   rebuildLoop(); },
    onAlpha:      (v) => { liveParams.alpha = v;         rebuildLoop(); },
    // Commands
    onKick:  (t) => loop.applyDisturbanceKick(t),
    onStart: () => {},
    onPause: () => {
      paused = !paused;
      loop.setPaused(paused);
      panel.setPaused(paused);
    },
    onReset: () => {
      rebuildLoop();
      lastFrameTimeMs = null;
    },
  },
});

// Header kick button
document.getElementById('btn-kick-hdr')!.addEventListener('click', () => {
  loop.applyDisturbanceKick(0.5);
});

// ---------------------------------------------------------------------------
// RAF render loop (R2.5 — physics steps inside tick(), NOT driven by RAF dt)
// ---------------------------------------------------------------------------

let frameCount = 0;
let fpsAccMs = 0;
let displayedFps = 0;

function rafLoop(nowMs: number): void {
  const realDtMs = lastFrameTimeMs !== null ? nowMs - lastFrameTimeMs : 0;
  lastFrameTimeMs = nowMs;

  // FPS counter
  fpsAccMs += realDtMs;
  frameCount++;
  if (fpsAccMs >= 500) {
    displayedFps = Math.round(frameCount / (fpsAccMs / 1000));
    frameCount = 0;
    fpsAccMs = 0;
    stFps.textContent = String(displayedFps);
  }

  // 1. Advance physics (accumulator pattern — R2.5)
  if (!paused && realDtMs > 0) {
    loop.tick(realDtMs);
  }

  // 2. Read snapshot (single read per frame — consistent state)
  const snap = loop.getSnapshot();

  // 3. Log to telemetry at ~50 Hz (charts) and append to export buffer
  logAccumulatorMs += realDtMs;
  if (logAccumulatorMs >= LOG_EVERY_MS) {
    logAccumulatorMs = 0;
    logger.log(snap);
    // Maintain a rolling export buffer (drop oldest when full)
    if (exportBuffer.length >= MAX_EXPORT_SNAPSHOTS) exportBuffer.shift();
    exportBuffer.push({ ...snap, trueState: { ...snap.trueState }, sensorReading: { ...snap.sensorReading } });
  }

  // 4. Update header readouts
  hdrTrue.textContent  = (snap.trueState.theta2 * RAD_TO_DEG).toFixed(2) + '°';
  hdrEst.textContent   = (snap.estimatedAngle   * RAD_TO_DEG).toFixed(2) + '°';
  hdrOmega.textContent = snap.trueState.omega2.toFixed(3) + ' r/s';
  hdrRpm.textContent   = (snap.trueState.omega1 * RAD_S_TO_RPM).toFixed(0) + ' RPM';
  stTime.textContent   = snap.simTimeSec.toFixed(2) + ' s';
  stCtrl.textContent   = snap.controlOutput.toFixed(4) + ' N·m';

  // Saturation indicator
  const saturated = Math.abs(snap.rawControlOutput) > Math.abs(snap.controlOutput) + 1e-4;
  satBadge.classList.toggle('visible', saturated);

  // 5. Render canvas
  renderer.render(snap);

  // 6. Render charts
  chartAngle.render();
  chartOmega.render();
  chartRpm.render();
  chartCtrl.render();
  chartErr.render();

  // 7. Update panel readouts
  panel.updateReadouts(snap);

  requestAnimationFrame(rafLoop);
}

// Kick off RAF loop
requestAnimationFrame(rafLoop);

// ---------------------------------------------------------------------------
// Export / Import button wiring (Phase 6 — PRD §6.8)
// ---------------------------------------------------------------------------

document.getElementById('btn-export-csv')!.addEventListener('click', () => {
  if (exportBuffer.length === 0) {
    alert('No telemetry data yet. Let the simulation run for a moment first.');
    return;
  }
  const ts = new Date().toISOString().replace(/[:.]/g, '-');
  exportTelemetryCSV(exportBuffer, `reaction_wheel_telemetry_${ts}.csv`);
});

document.getElementById('btn-export-json')!.addEventListener('click', () => {
  const ts = new Date().toISOString().replace(/[:.]/g, '-');
  exportConfigJSON(simConfig, `reaction_wheel_config_${ts}.json`);
});

document.getElementById('file-import-input')!.addEventListener('change', async (evt) => {
  const input = evt.target as HTMLInputElement;
  const file  = input.files?.[0];
  if (!file) return;
  try {
    const importedConfig = await importConfigJSON(file);
    // Apply the imported config: update liveParams to match, then rebuild loop
    liveParams.i1           = importedConfig.physicsParams.I1;
    liveParams.i2           = importedConfig.physicsParams.I2;
    liveParams.friction1    = importedConfig.physicsParams.frictionCoeff1;
    liveParams.friction2    = importedConfig.physicsParams.frictionCoeff2;
    liveParams.maxRpm       = importedConfig.motorParams.maxRPM;
    liveParams.maxTorqueNm  = importedConfig.motorParams.maxTorqueNm;
    liveParams.motorTauS    = importedConfig.motorParams.timeConstantS;
    liveParams.gyroNoiseSigma       = importedConfig.sensorParams.gyroNoiseSigma;
    liveParams.accelAngleNoiseSigma = importedConfig.sensorParams.accelAngleNoiseSigma;
    liveParams.alpha        = importedConfig.estimatorParams.alpha;
    liveParams.kp           = importedConfig.pidParams.kp;
    liveParams.ki           = importedConfig.pidParams.ki;
    liveParams.kd           = importedConfig.pidParams.kd;
    liveParams.setpointRad  = importedConfig.initialSetpoint ?? 0;
    // Rebuild using the exact imported config (preserves sensor seed for reproducibility)
    simConfig = importedConfig;
    loop = new SimulationLoop(simConfig);
    loop.setSetpoint(liveParams.setpointRad);
    logger.clearAll();
    exportBuffer.length = 0;
    lastFrameTimeMs = null;
    console.info('[Import] Config loaded successfully:', file.name);
  } catch (err) {
    alert(`Import failed: ${(err as Error).message}`);
  }
  // Reset file input so the same file can be re-imported
  input.value = '';
});
