/**
 * main.ts
 *
 * Application entry point — wires all UI modules and starts the simulation.
 *
 * Responsibilities:
 *   1. Construct the SimulationLoop with default parameters.
 *   2. Mount CanvasRenderer, TelemetryChart, and ControlPanel into the DOM.
 *   3. Start the requestAnimationFrame render loop.
 *   4. In each RAF frame: advance the simulation via tick(realDtMs),
 *      then render (CanvasRenderer.render, TelemetryChart.render).
 *
 * Critical: the physics steps are driven by SimulationLoop.tick(realDtMs) which
 * uses an internal accumulator at PHYSICS_DT_S. The RAF frame delta DOES NOT
 * directly drive physics timesteps (R2.5, PRD §6.1).
 *
 * R2.3 — main.ts only interacts with UI module classes and SimulationLoop.
 *         It does NOT import physicsEngine, pidController, or core modules.
 *
 * Phase 0: basic page structure only — shows a placeholder message on an empty page.
 *          The full wiring will be implemented in Phase 5.
 */

// Intentionally minimal in Phase 0: just enough to make `npm run dev` boot
// without errors and display a recognizable placeholder page.

const app = document.querySelector<HTMLDivElement>('#app');

if (app) {
  app.innerHTML = `
    <div style="
      font-family: system-ui, sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      min-height: 100vh;
      background: #0f0f13;
      color: #e0e0e8;
      gap: 1rem;
    ">
      <h1 style="font-size: 1.8rem; margin: 0; color: #a78bfa;">
        ⚙️ Reaction Wheel Attitude Control Simulator
      </h1>
      <p style="color: #888; margin: 0; font-size: 0.95rem;">
        Phase 0 — scaffold complete. UI will be implemented in Phase 5.
      </p>
      <p style="color: #555; font-size: 0.8rem; font-family: monospace;">
        Run <code style="color: #6ee7b7;">npm test</code> to verify module contracts.
      </p>
    </div>
  `;
}
