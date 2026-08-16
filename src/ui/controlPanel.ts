/**
 * controlPanel.ts
 *
 * Live control panel — sliders and inputs bound to simulation parameters.
 *
 * Provides UI controls for (PRD §6.6):
 *   - Target angle input
 *   - Kp, Ki, Kd gain sliders/inputs
 *   - I₁ (wheel inertia) and I₂ (body inertia) sliders
 *   - Friction coefficient input
 *   - Disturbance kick button
 *   - Start / Pause / Reset controls
 *
 * Changes apply on the next control tick — no page reload required.
 *
 * R2.3 — This module only calls SimulationLoop's public API.
 *         It does NOT import physicsEngine, pidController, or core modules.
 * R1.3 — UI inputs in user-friendly units (degrees, RPM) are converted to SI
 *         before being passed to the simulation — conversion happens here, at
 *         the display boundary, not inside any core module.
 *
 * Phase 0: class skeleton and method signatures only. Bodies throw 'not implemented'.
 * Phase 5 will implement DOM construction and event binding.
 */

import type { SimulationLoop } from '../sim/simulationLoop.ts';

/**
 * Initial/default values for all control panel inputs.
 * Used to populate the panel on first render and after reset.
 */
export interface ControlPanelDefaults {
  /** Target body angle [deg]. Displayed in degrees; passed to sim in rad. */
  targetAngleDeg: number;
  kp: number;
  ki: number;
  kd: number;
  /** Wheel inertia I₁ [kg·m²]. */
  I1: number;
  /** Body inertia I₂ [kg·m²]. */
  I2: number;
  /** Bearing friction coefficient [N·m·s/rad]. */
  frictionCoeff: number;
  /** Disturbance kick torque magnitude [N·m]. Applied on button press. */
  kickTorqueNm: number;
}

/**
 * ControlPanel — manages the live parameter control UI.
 *
 * Constructs and mounts DOM elements, wires event listeners, and
 * calls the SimulationLoop's public command API on changes.
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 5.
 */
export class ControlPanel {
  private readonly container: HTMLElement;
  private readonly sim: SimulationLoop;
  private readonly defaults: ControlPanelDefaults;

  constructor(container: HTMLElement, sim: SimulationLoop, defaults: ControlPanelDefaults) {
    this.container = container;
    this.sim = sim;
    this.defaults = defaults;
    void this.container;
    void this.sim;
    void this.defaults;
  }

  /**
   * Builds and mounts all control panel DOM elements into the container.
   * Wires all event listeners to the SimulationLoop.
   *
   * @throws Error('not implemented')
   */
  mount(): void {
    throw new Error('not implemented');
  }

  /**
   * Resets all panel inputs to the default values.
   * Called when the simulation is reset.
   *
   * @throws Error('not implemented')
   */
  resetToDefaults(): void {
    throw new Error('not implemented');
  }

  /**
   * Updates display-only readouts (e.g., current true angle, estimated angle).
   * Called once per render frame from main.ts's RAF loop.
   *
   * @throws Error('not implemented')
   */
  updateReadouts(): void {
    throw new Error('not implemented');
  }
}
