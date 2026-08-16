/**
 * telemetryChart.ts
 *
 * Scrolling telemetry strip charts — live time-series visualization.
 *
 * Displays channels (PRD §6.6, minimum):
 *   - True body angle [deg display, rad internal]
 *   - Estimated body angle [deg display, rad internal]
 *   - Body angular velocity [rad/s or deg/s display]
 *   - Wheel RPM [display only — converted from rad/s at boundary]
 *   - Control output [N·m or duty cycle]
 *   - Error (setpoint − estimated) [deg display, rad internal]
 *
 * Implemented as a hand-rolled scrolling canvas strip chart (PRD §8: avoid heavy
 * charting library dependencies; full control is preferred here).
 *
 * R2.3 — Reads data only from SimulationLoop's public API (SimSnapshot ring buffer).
 *         No direct import of core physics/control modules.
 * R1.3 — Receives values in SI units; converts to display units at the render boundary.
 *
 * Phase 0: class skeleton and method signatures only. Bodies throw 'not implemented'.
 * Phase 5 will implement the full scrolling chart rendering.
 */

import type { SimSnapshot } from '../sim/simulationLoop.ts';

/**
 * Configuration for one chart channel (one strip).
 */
export interface ChartChannelConfig {
  /** Display label for this channel. */
  label: string;

  /** Display color (CSS color string). */
  color: string;

  /** Y-axis minimum value in display units. */
  yMin: number;

  /** Y-axis maximum value in display units. */
  yMax: number;

  /** Function extracting the channel's value from a SimSnapshot (in display units). */
  getValue: (snapshot: SimSnapshot) => number;
}

/**
 * Configuration for the telemetry chart panel.
 */
export interface TelemetryChartConfig {
  /** Canvas element to render the charts into. */
  canvas: HTMLCanvasElement;

  /** How many seconds of history to display (x-axis window) [s]. */
  windowSec: number;

  /** Maximum number of data points to keep in the ring buffer. */
  maxPoints: number;
}

/**
 * TelemetryChart — scrolling multi-channel strip chart.
 *
 * Accumulates SimSnapshots into a ring buffer and renders them as scrolling
 * strip charts. Each channel occupies a horizontal band of the canvas.
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 5.
 */
export class TelemetryChart {
  private readonly config: TelemetryChartConfig;
  private readonly channels: ChartChannelConfig[];

  constructor(config: TelemetryChartConfig, channels: ChartChannelConfig[]) {
    this.config = config;
    this.channels = channels;
    void this.config;
    void this.channels;
  }

  /**
   * Pushes a new data point into the ring buffer.
   * Call once per simulation tick (or at a logging interval).
   *
   * @param snapshot  Current simulation snapshot to log.
   *
   * @throws Error('not implemented')
   */
  push(_snapshot: SimSnapshot): void {
    throw new Error('not implemented');
  }

  /**
   * Renders the current ring buffer to the canvas.
   * Called once per requestAnimationFrame tick.
   *
   * @throws Error('not implemented')
   */
  render(): void {
    throw new Error('not implemented');
  }

  /**
   * Clears all buffered data (called on simulation reset).
   *
   * @throws Error('not implemented')
   */
  clear(): void {
    throw new Error('not implemented');
  }
}
