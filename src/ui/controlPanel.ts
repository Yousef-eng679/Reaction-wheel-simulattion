/**
 * controlPanel.ts
 *
 * Live control panel — all sliders, inputs, and buttons for the simulation UI (PRD §6.6).
 *
 * Responsibilities:
 *   - Render the HTML control panel into a container element.
 *   - Bind all inputs to live callbacks (called immediately on change — no page reload).
 *   - Expose an updateReadouts() method that the RAF loop calls each frame to keep
 *     numeric readouts up to date.
 *
 * R2.3 — ControlPanel calls SimulationLoop's public API only (via callbacks).
 *         It has zero imports from core modules.
 */

export interface ControlPanelCallbacks {
  onSetpoint: (rad: number) => void;
  onKp: (v: number) => void;
  onKi: (v: number) => void;
  onKd: (v: number) => void;
  onI1: (v: number) => void;
  onI2: (v: number) => void;
  onFriction1: (v: number) => void;
  onFriction2: (v: number) => void;
  onMaxRpm: (v: number) => void;
  onMaxTorque: (v: number) => void;
  onMotorTau: (v: number) => void;
  onGyroNoise: (v: number) => void;
  onAccelNoise: (v: number) => void;
  onAlpha: (v: number) => void;
  onKick: (torqueNm: number) => void;
  onStart: () => void;
  onPause: () => void;
  onReset: () => void;
}

export interface ControlPanelConfig {
  container: HTMLElement;
  callbacks: ControlPanelCallbacks;
  // Initial / default values
  setpointDeg: number;
  kp: number; ki: number; kd: number;
  i1: number; i2: number;
  friction1: number; friction2: number;
  maxRpm: number; maxTorqueNm: number; motorTauS: number;
  gyroNoiseSigma: number; accelAngleNoiseSigma: number; alpha: number;
}

/** Helper to create a slider + numeric readout row. Returns the input element. */
function makeParam(
  container: HTMLElement,
  label: string,
  unit: string,
  min: number,
  max: number,
  step: number,
  value: number,
  displayFn: (v: number) => string,
  onChange: (v: number) => void,
): HTMLInputElement {
  const wrap = document.createElement('div');
  wrap.className = 'param';

  const header = document.createElement('div');
  header.className = 'param-header';

  const lbl = document.createElement('span');
  lbl.className = 'param-label';
  lbl.textContent = label;

  const unitSpan = document.createElement('span');
  unitSpan.className = 'param-unit';
  unitSpan.textContent = unit;

  const valSpan = document.createElement('span');
  valSpan.className = 'param-value';
  valSpan.textContent = displayFn(value);

  header.appendChild(lbl);
  const rightSide = document.createElement('div');
  rightSide.style.display = 'flex';
  rightSide.style.alignItems = 'baseline';
  rightSide.style.gap = '0.4rem';
  rightSide.appendChild(unitSpan);
  rightSide.appendChild(valSpan);
  header.appendChild(rightSide);

  const slider = document.createElement('input');
  slider.type = 'range';
  slider.min = String(min);
  slider.max = String(max);
  slider.step = String(step);
  slider.value = String(value);

  slider.addEventListener('input', () => {
    const v = Number(slider.value);
    valSpan.textContent = displayFn(v);
    onChange(v);
  });

  wrap.appendChild(header);
  wrap.appendChild(slider);
  container.appendChild(wrap);
  return slider;
}

function makeSectionTitle(container: HTMLElement, text: string): void {
  const el = document.createElement('div');
  el.className = 'section-title';
  el.textContent = text;
  container.appendChild(el);
}

export class ControlPanel {
  private readonly container: HTMLElement;
  private readonly cbs: ControlPanelCallbacks;

  // Elements to update from RAF readouts
  private elTrueAngle!: HTMLElement;
  private elEstAngle!: HTMLElement;
  private elOmega2!: HTMLElement;
  private elWheelRpm!: HTMLElement;
  private elSimTime!: HTMLElement;
  private elCtrlOut!: HTMLElement;
  private elSatBadge!: HTMLElement;
  private elBtnStart!: HTMLButtonElement;
  private elBtnPause!: HTMLButtonElement;

  private readonly RAD_TO_DEG = 180 / Math.PI;
  private readonly RAD_S_TO_RPM = 60 / (2 * Math.PI);

  constructor(config: ControlPanelConfig) {
    this.container = config.container;
    this.cbs = config.callbacks;
    this._build(config);
  }

  /**
   * Call from RAF loop each frame with the latest SimSnapshot to update numeric readouts.
   */
  updateReadouts(snap: {
    trueState: { theta2: number; omega2: number; omega1: number };
    estimatedAngle: number;
    controlOutput: number;
    rawControlOutput: number;
    simTimeSec: number;
  }): void {
    this.elTrueAngle.textContent = (snap.trueState.theta2 * this.RAD_TO_DEG).toFixed(2) + '°';
    this.elEstAngle.textContent  = (snap.estimatedAngle   * this.RAD_TO_DEG).toFixed(2) + '°';
    this.elOmega2.textContent    = snap.trueState.omega2.toFixed(3) + ' r/s';
    this.elWheelRpm.textContent  = (snap.trueState.omega1 * this.RAD_S_TO_RPM).toFixed(0) + ' RPM';
    this.elSimTime.textContent   = snap.simTimeSec.toFixed(2) + ' s';
    this.elCtrlOut.textContent   = (snap.controlOutput * 1000).toFixed(2) + ' mN·m';

    // Saturation: raw > clamped + small epsilon
    const saturated = Math.abs(snap.rawControlOutput) > Math.abs(snap.controlOutput) + 1e-4;
    this.elSatBadge.classList.toggle('visible', saturated);
  }

  /** Flips start/pause button state. */
  setPaused(paused: boolean): void {
    this.elBtnStart.textContent = paused ? '▶ Resume' : '⏸ Pause';
  }

  // ---------------------------------------------------------------------------
  // Private DOM construction
  // ---------------------------------------------------------------------------

  private _build(cfg: ControlPanelConfig): void {
    this.container.innerHTML = '';
    const DEG = (r: number) => (r * this.RAD_TO_DEG).toFixed(1) + '°';
    const SCI = (v: number) => v.toExponential(1);
    const F3  = (v: number) => v.toFixed(3);
    const F2  = (v: number) => v.toFixed(2);

    // ── Setpoint ──
    makeSectionTitle(this.container, 'Target');
    makeParam(this.container, 'Target angle', 'deg', -180, 180, 1, cfg.setpointDeg,
      v => v.toFixed(1) + '°',
      v => cfg.callbacks.onSetpoint(v * Math.PI / 180),
    );

    // ── PID Gains ──
    makeSectionTitle(this.container, 'PID Gains');
    makeParam(this.container, 'Kp', 'N·m/rad', 0, 5, 0.05, cfg.kp, F3, v => cfg.callbacks.onKp(v));
    makeParam(this.container, 'Ki', 'N·m/(rad·s)', 0, 2, 0.01, cfg.ki, F3, v => cfg.callbacks.onKi(v));
    makeParam(this.container, 'Kd', 'N·m·s/rad', 0, 2, 0.01, cfg.kd, F3, v => cfg.callbacks.onKd(v));

    // ── Inertia ──
    makeSectionTitle(this.container, 'Inertia');
    makeParam(this.container, 'I₁ (wheel)', 'kg·m²', 0.00005, 0.001, 0.00001, cfg.i1, SCI, v => cfg.callbacks.onI1(v));
    makeParam(this.container, 'I₂ (body)',  'kg·m²', 0.001,   0.05,  0.001,   cfg.i2, SCI, v => cfg.callbacks.onI2(v));

    // ── Friction ──
    makeSectionTitle(this.container, 'Friction');
    makeParam(this.container, 'Wheel friction', 'N·m·s/r', 0, 0.001, 0.00001, cfg.friction1, SCI, v => cfg.callbacks.onFriction1(v));
    makeParam(this.container, 'Body friction',  'N·m·s/r', 0, 0.001, 0.00001, cfg.friction2, SCI, v => cfg.callbacks.onFriction2(v));

    // ── Motor ──
    makeSectionTitle(this.container, 'Motor (Actuator)');
    makeParam(this.container, 'Max RPM', 'RPM', 1000, 10000, 100, cfg.maxRpm, v => v.toFixed(0), v => cfg.callbacks.onMaxRpm(v));
    makeParam(this.container, 'Max torque', 'N·m', 0.001, 0.5, 0.001, cfg.maxTorqueNm, F3, v => cfg.callbacks.onMaxTorque(v));
    makeParam(this.container, 'Lag τ', 's', 0.01, 1, 0.01, cfg.motorTauS, F2, v => cfg.callbacks.onMotorTau(v));

    // ── Sensor ──
    makeSectionTitle(this.container, 'Sensor (IMU)');
    makeParam(this.container, 'Gyro noise σ', 'rad/s', 0, 0.1, 0.001, cfg.gyroNoiseSigma, F3, v => cfg.callbacks.onGyroNoise(v));
    makeParam(this.container, 'Accel noise σ', 'rad', 0, 0.2, 0.005, cfg.accelAngleNoiseSigma, F3, v => cfg.callbacks.onAccelNoise(v));
    makeParam(this.container, 'Filter α', '', 0.8, 0.999, 0.001, cfg.alpha, v => v.toFixed(3), v => cfg.callbacks.onAlpha(v));

    // ── Disturbance ──
    makeSectionTitle(this.container, 'Disturbance');

    const kickWrap = document.createElement('div');
    kickWrap.style.display = 'flex';
    kickWrap.style.gap = '0.5rem';
    kickWrap.style.alignItems = 'center';

    const kickInput = document.createElement('input');
    kickInput.type = 'number';
    kickInput.value = '0.5';
    kickInput.step = '0.1';
    kickInput.min = '-5';
    kickInput.max = '5';
    kickInput.style.width = '70px';
    kickInput.title = 'Disturbance torque [N·m]';

    const kickLabel = document.createElement('span');
    kickLabel.className = 'param-unit';
    kickLabel.textContent = 'N·m';
    kickLabel.style.fontSize = '0.75rem';
    kickLabel.style.color = 'var(--text-dim)';

    const kickBtn = document.createElement('button');
    kickBtn.className = 'warn';
    kickBtn.textContent = '💥 Kick';
    kickBtn.style.marginLeft = 'auto';
    kickBtn.title = 'Apply an instantaneous torque disturbance to the body';
    kickBtn.addEventListener('click', () => {
      const t = Number(kickInput.value);
      if (Number.isFinite(t)) cfg.callbacks.onKick(t);
    });

    kickWrap.appendChild(kickInput);
    kickWrap.appendChild(kickLabel);
    kickWrap.appendChild(kickBtn);
    this.container.appendChild(kickWrap);

    // ── Readouts section ──
    makeSectionTitle(this.container, 'Live Readouts');
    this.elTrueAngle = this._makeReadout('True angle',  'true');
    this.elEstAngle  = this._makeReadout('Estimated',   'est');
    this.elOmega2    = this._makeReadout('ω₂ (body)',   'omega');
    this.elWheelRpm  = this._makeReadout('Wheel speed', 'rpm');
    this.elSimTime   = this._makeReadout('Sim time',    '');
    this.elCtrlOut   = this._makeReadout('Control out', 'ctrl');

    // Saturation badge inside the panel
    this.elSatBadge = document.createElement('div');
    this.elSatBadge.className = 'sat-badge';
    this.elSatBadge.textContent = '⚡ SATURATED';
    this.elSatBadge.style.display = 'inline-block';
    this.elSatBadge.style.visibility = 'hidden';
    this.elSatBadge.style.position = 'static';
    this.elSatBadge.style.marginTop = '0.3rem';
    this.container.appendChild(this.elSatBadge);

    // ── Controls ──
    makeSectionTitle(this.container, 'Simulation');
    const ctrlRow = document.createElement('div');
    ctrlRow.style.display = 'flex';
    ctrlRow.style.gap = '0.4rem';
    ctrlRow.style.flexWrap = 'wrap';

    this.elBtnStart = document.createElement('button');
    this.elBtnStart.className = 'primary';
    this.elBtnStart.textContent = '⏸ Pause';
    this.elBtnStart.addEventListener('click', () => {
      cfg.callbacks.onPause();
    });

    const btnReset = document.createElement('button');
    btnReset.textContent = '↺ Reset';
    btnReset.addEventListener('click', () => cfg.callbacks.onReset());

    ctrlRow.appendChild(this.elBtnStart);
    ctrlRow.appendChild(btnReset);
    this.container.appendChild(ctrlRow);
  }

  private _makeReadout(label: string, cls: string): HTMLElement {
    const row = document.createElement('div');
    row.style.display = 'flex';
    row.style.justifyContent = 'space-between';
    row.style.alignItems = 'baseline';
    row.style.padding = '0.1rem 0';

    const lbl = document.createElement('span');
    lbl.className = 'param-label';
    lbl.textContent = label;

    const val = document.createElement('span');
    val.className = 'readout-value ' + cls;
    val.textContent = '—';
    val.style.fontFamily = '"JetBrains Mono", monospace';
    val.style.fontSize = '0.82rem';

    row.appendChild(lbl);
    row.appendChild(val);
    this.container.appendChild(row);
    return val;
  }
}
