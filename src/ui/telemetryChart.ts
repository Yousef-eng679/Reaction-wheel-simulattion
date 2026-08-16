/**
 * telemetryChart.ts
 *
 * Scrolling strip-chart for real-time telemetry display (PRD §6.6).
 *
 * Each TelemetryChart instance manages one canvas that shows one or more time-series
 * signals over a rolling window. Data is stored in a fixed-size ring buffer. The chart
 * scrolls leftward as new data arrives (classic strip-chart pattern used in hardware
 * telemetry tools).
 *
 * Signals rendered:
 *   - Chart 1: True angle (purple) + Estimated angle (green)    [rad → deg for display]
 *   - Chart 2: Body angular velocity ω₂ (amber)                 [rad/s]
 *   - Chart 3: Wheel RPM (blue)                                 [rpm]
 *   - Chart 4: Control output (red) + Raw control output (dim)  [N·m]
 *   - Chart 5: Error (orange)                                    [rad → deg]
 *
 * R2.3 — reads only SimSnapshot fields. No imports from core modules.
 * R4.4 — hand-rolled canvas chart; no external charting dependencies.
 */

export interface SignalConfig {
  /** Key to read from a data record (matches TelemetryRecord fields). */
  key: keyof TelemetryRecord;
  /** Line color (CSS color string). */
  color: string;
  /** Line width [px]. */
  lineWidth?: number;
  /** Draw as a dashed line. */
  dashed?: boolean;
}

/** One logged telemetry sample. All values already converted to display units. */
export interface TelemetryRecord {
  simTimeSec: number;
  trueAngleDeg: number;
  estimatedAngleDeg: number;
  omega2RadS: number;
  wheelRpm: number;
  controlOutputNm: number;
  rawControlOutputNm: number;
  errorDeg: number;
}

export interface ChartConfig {
  canvas: HTMLCanvasElement;
  signals: SignalConfig[];
  /** Y-axis label shown on the left. */
  yLabel: string;
  /** Fixed Y range. If null, auto-scales. */
  yMin: number;
  yMax: number;
  /** Number of samples to show in the rolling window. */
  windowSamples: number;
}

export class TelemetryChart {
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private readonly config: ChartConfig;

  /** Ring buffer of telemetry records. */
  private readonly buffer: TelemetryRecord[];
  private bufferHead: number = 0;
  private bufferCount: number = 0;

  constructor(config: ChartConfig) {
    this.config = config;
    this.canvas = config.canvas;
    this.ctx = config.canvas.getContext('2d')!;
    this.buffer = new Array(config.windowSamples).fill(null);
  }

  /** Push a new telemetry record into the ring buffer. */
  push(record: TelemetryRecord): void {
    this.buffer[this.bufferHead] = record;
    this.bufferHead = (this.bufferHead + 1) % this.config.windowSamples;
    if (this.bufferCount < this.config.windowSamples) this.bufferCount++;
  }

  /** Clears all buffered data. */
  clear(): void {
    this.bufferCount = 0;
    this.bufferHead = 0;
  }

  /** Renders the current buffer as a scrolling strip chart. */
  render(): void {
    const { ctx, config } = this;
    const W = this.canvas.width;
    const H = this.canvas.height;
    const PAD_L = 36, PAD_R = 6, PAD_T = 6, PAD_B = 18;
    const plotW = W - PAD_L - PAD_R;
    const plotH = H - PAD_T - PAD_B;

    // --- Background ---
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = '#0d0d11';
    ctx.fillRect(0, 0, W, H);

    // --- Compute visible Y range ---
    let yMin = config.yMin, yMax = config.yMax;
    if (this.bufferCount > 0) {
      // If yMin === yMax (auto hint), compute from data
      if (yMin === yMax) {
        yMin = Infinity; yMax = -Infinity;
        this._eachSample((_i, rec) => {
          for (const sig of config.signals) {
            const v = rec[sig.key] as number;
            if (v < yMin) yMin = v;
            if (v > yMax) yMax = v;
          }
        });
        const pad = Math.max(Math.abs(yMax - yMin) * 0.15, 0.01);
        yMin -= pad; yMax += pad;
      }
    }
    const yRange = yMax - yMin || 1;

    // --- Grid lines ---
    const gridCount = 4;
    ctx.strokeStyle = 'rgba(255,255,255,0.05)';
    ctx.lineWidth = 1;
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.fillStyle = 'rgba(255,255,255,0.25)';
    ctx.textAlign = 'right';

    for (let g = 0; g <= gridCount; g++) {
      const frac = g / gridCount;
      const yPx = PAD_T + plotH * (1 - frac);
      ctx.beginPath();
      ctx.moveTo(PAD_L, yPx);
      ctx.lineTo(PAD_L + plotW, yPx);
      ctx.stroke();

      const val = yMin + yRange * frac;
      ctx.fillText(val.toFixed(1), PAD_L - 4, yPx + 3);
    }

    // Zero line (if zero is in range)
    if (yMin < 0 && yMax > 0) {
      const yZero = PAD_T + plotH * (1 - (0 - yMin) / yRange);
      ctx.strokeStyle = 'rgba(255,255,255,0.12)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(PAD_L, yZero);
      ctx.lineTo(PAD_L + plotW, yZero);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // --- Y label ---
    ctx.save();
    ctx.translate(10, PAD_T + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillStyle = 'rgba(255,255,255,0.2)';
    ctx.font = '8px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText(config.yLabel, 0, 0);
    ctx.restore();

    if (this.bufferCount < 2) return;

    // --- Plot each signal ---
    const n = this.bufferCount;

    for (const sig of config.signals) {
      ctx.beginPath();
      ctx.strokeStyle = sig.color;
      ctx.lineWidth = sig.lineWidth ?? 1.5;
      if (sig.dashed) ctx.setLineDash([4, 3]);
      else ctx.setLineDash([]);

      let firstPoint = true;
      this._eachSample((i, rec) => {
        const xFrac = i / (Math.min(n, config.windowSamples) - 1);
        const val   = rec[sig.key] as number;
        const xPx   = PAD_L + xFrac * plotW;
        const yPx   = PAD_T + plotH * (1 - (val - yMin) / yRange);
        if (firstPoint) { ctx.moveTo(xPx, yPx); firstPoint = false; }
        else ctx.lineTo(xPx, yPx);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Current values at right edge
    ctx.textAlign = 'left';
    const lastRec = this._lastRecord();
    if (lastRec) {
      config.signals.forEach((sig, i) => {
        const val = lastRec[sig.key] as number;
        ctx.fillStyle = sig.color;
        ctx.font = 'bold 9px "JetBrains Mono", monospace';
        ctx.fillText(val.toFixed(2), PAD_L + plotW + 2, PAD_T + 10 + i * 12);
      });
    }
  }

  // ---------------------------------------------------------------------------
  // Private helpers
  // ---------------------------------------------------------------------------

  /** Iterates samples in chronological order, calling cb(index_0_to_N-1, record). */
  private _eachSample(cb: (i: number, rec: TelemetryRecord) => void): void {
    const n = this.bufferCount;
    const start = n < this.config.windowSamples
      ? 0
      : this.bufferHead; // oldest entry

    for (let j = 0; j < n; j++) {
      const idx = (start + j) % this.config.windowSamples;
      if (this.buffer[idx]) cb(j, this.buffer[idx]);
    }
  }

  private _lastRecord(): TelemetryRecord | null {
    if (!this.bufferCount) return null;
    const idx = (this.bufferHead - 1 + this.config.windowSamples) % this.config.windowSamples;
    return this.buffer[idx];
  }
}

// ---------------------------------------------------------------------------
// TelemetryLogger — populates TelemetryChart buffers each physics step
// ---------------------------------------------------------------------------

/**
 * TelemetryLogger: takes a SimSnapshot, converts to display units, and pushes
 * to all registered charts. Decoupled from render — logging runs at physics
 * or sensor rate; rendering runs at RAF rate.
 */
export class TelemetryLogger {
  private readonly charts: TelemetryChart[];
  private readonly RAD_TO_DEG = 180 / Math.PI;
  private readonly RAD_S_TO_RPM = 60 / (2 * Math.PI);

  constructor(charts: TelemetryChart[]) {
    this.charts = charts;
  }

  log(snap: {
    simTimeSec: number;
    trueState: { theta2: number; omega2: number; omega1: number };
    estimatedAngle: number;
    controlOutput: number;
    rawControlOutput: number;
    setpoint: number;
  }): void {
    const record: TelemetryRecord = {
      simTimeSec:         snap.simTimeSec,
      trueAngleDeg:       snap.trueState.theta2 * this.RAD_TO_DEG,
      estimatedAngleDeg:  snap.estimatedAngle   * this.RAD_TO_DEG,
      omega2RadS:         snap.trueState.omega2,
      wheelRpm:           snap.trueState.omega1  * this.RAD_S_TO_RPM,
      controlOutputNm:    snap.controlOutput,
      rawControlOutputNm: snap.rawControlOutput,
      errorDeg:           (snap.setpoint - snap.trueState.theta2) * this.RAD_TO_DEG,
    };
    for (const chart of this.charts) chart.push(record);
  }

  clearAll(): void {
    for (const chart of this.charts) chart.clear();
  }
}
