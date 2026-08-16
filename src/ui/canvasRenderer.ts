/**
 * canvasRenderer.ts
 *
 * 2D Canvas renderer — top-down visualization of the reaction wheel system.
 *
 * Renders (PRD §6.6):
 *   - Spacecraft body as a shape, rotating at its true simulated angle θ₂.
 *   - Reaction wheel inside the body, rotating at its true simulated speed ω₁.
 *   - Target angle marker.
 *   - Error arc between current and target angle.
 *
 * R2.3 — This module reads only from SimulationLoop's public API (getSnapshot()).
 *         It must NEVER import physicsEngine, pidController, or other core modules.
 * R1.3 — Receives state in SI units; converts to display units (degrees) only here,
 *         at the display boundary.
 *
 * Phase 0: class skeleton and method signatures only. Bodies throw 'not implemented'.
 * Phase 5 will implement the full Canvas rendering.
 */

import type { SimSnapshot } from '../sim/simulationLoop.ts';

/**
 * Configuration for the canvas renderer.
 */
export interface RendererConfig {
  /** HTML Canvas element to render into. */
  canvas: HTMLCanvasElement;

  /** Pixel width of the canvas. */
  width: number;

  /** Pixel height of the canvas. */
  height: number;
}

/**
 * CanvasRenderer — owns the 2D rendering context and knows how to draw the system.
 *
 * Rendering is driven externally by main.ts's requestAnimationFrame loop.
 * The renderer reads the latest SimSnapshot each frame — it does not step physics.
 *
 * @throws Error('not implemented') — Phase 0 stub; will be implemented in Phase 5.
 */
export class CanvasRenderer {
  private readonly config: RendererConfig;

  constructor(config: RendererConfig) {
    this.config = config;
    void this.config;
  }

  /**
   * Renders one frame to the canvas using the given simulation snapshot.
   * Called once per requestAnimationFrame tick.
   *
   * @param snapshot  Current simulation state from SimulationLoop.getSnapshot().
   *
   * @throws Error('not implemented')
   */
  render(_snapshot: SimSnapshot): void {
    throw new Error('not implemented');
  }

  /**
   * Resizes the canvas (called on window resize events).
   *
   * @throws Error('not implemented')
   */
  resize(_width: number, _height: number): void {
    throw new Error('not implemented');
  }
}
