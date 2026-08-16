/**
 * canvasRenderer.ts
 *
 * 2D top-down Canvas renderer for the reaction wheel simulation (PRD §6.6).
 *
 * Renders:
 *   - Spacecraft body: large disc rotating at trueState.theta2
 *   - Reaction wheel: smaller disc inside body, rotating at trueState.theta1 (much faster visually)
 *   - Target angle marker: a dashed radial line showing the setpoint
 *   - Error arc: a shaded arc between current body angle and target
 *   - Body angle reference arm: a solid line showing the body's pointing direction
 *   - Estimated angle arm: a dimmer line showing what the estimator believes
 *
 * R2.3 — reads only from SimSnapshot (the loop's public API). Never imports core modules.
 */

export interface CanvasRendererOptions {
  /** Canvas element to draw into. */
  canvas: HTMLCanvasElement;
  /** Physical width and height to use for the drawing area [px]. */
  width: number;
  height: number;
}

export class CanvasRenderer {
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private width: number;
  private height: number;

  constructor(options: CanvasRendererOptions) {
    this.canvas = options.canvas;
    this.ctx = options.canvas.getContext('2d')!;
    this.width = options.width;
    this.height = options.height;
    this.canvas.width = options.width;
    this.canvas.height = options.height;
  }

  resize(width: number, height: number): void {
    this.width = width;
    this.height = height;
    this.canvas.width = width;
    this.canvas.height = height;
  }

  /**
   * Renders one frame from a SimSnapshot.
   * Called once per requestAnimationFrame. Not tied to physics rate (R2.5).
   */
  render(snap: {
    trueState: { theta1: number; theta2: number; omega1: number; omega2: number };
    estimatedAngle: number;
    setpoint: number;
    controlOutput: number;
    rawControlOutput: number;
  }): void {
    const { ctx } = this;
    const W = this.width;
    const H = this.height;
    const cx = W / 2;
    const cy = H / 2;

    // Radii scaled to canvas size
    const bodyR  = Math.min(W, H) * 0.32;
    const wheelR = bodyR * 0.40;
    const hubR   = wheelR * 0.18;

    // --- Background ---
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = '#0d0d11';
    ctx.fillRect(0, 0, W, H);

    // Faint grid rings
    ctx.strokeStyle = 'rgba(255,255,255,0.04)';
    ctx.lineWidth = 1;
    for (let r = bodyR * 0.5; r <= bodyR * 1.6; r += bodyR * 0.4) {
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.stroke();
    }

    // --- Error arc (between current body and target) ---
    const bodyAngle   = snap.trueState.theta2; // rad — 0 = up (+Y axis)
    const targetAngle = snap.setpoint;
    const errArcStart = -(Math.PI / 2) + bodyAngle;
    const errArcEnd   = -(Math.PI / 2) + targetAngle;
    const errorSign   = targetAngle > bodyAngle ? 1 : -1;

    ctx.beginPath();
    ctx.arc(cx, cy, bodyR + 14, errArcStart, errArcEnd, errorSign < 0);
    ctx.strokeStyle = 'rgba(249,115,22,0.55)';
    ctx.lineWidth = 4;
    ctx.stroke();

    // --- Target angle marker ---
    const tAngleCanvas = -(Math.PI / 2) + targetAngle;
    const markerLen = bodyR + 30;
    ctx.save();
    ctx.setLineDash([6, 5]);
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = 'rgba(124,110,255,0.6)';
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(tAngleCanvas) * markerLen, cy + Math.sin(tAngleCanvas) * markerLen);
    ctx.stroke();
    ctx.restore();

    // Target dot
    ctx.beginPath();
    ctx.arc(
      cx + Math.cos(tAngleCanvas) * (bodyR + 18),
      cy + Math.sin(tAngleCanvas) * (bodyR + 18),
      5, 0, Math.PI * 2,
    );
    ctx.fillStyle = '#7c6eff';
    ctx.fill();

    // --- Spacecraft body ---
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-(Math.PI / 2) + bodyAngle);

    // Body shadow
    ctx.shadowColor = 'rgba(124,110,255,0.15)';
    ctx.shadowBlur = 20;

    // Body disc
    const bodyGrad = ctx.createRadialGradient(0, 0, bodyR * 0.2, 0, 0, bodyR);
    bodyGrad.addColorStop(0, '#22223a');
    bodyGrad.addColorStop(1, '#15151f');
    ctx.beginPath();
    ctx.arc(0, 0, bodyR, 0, Math.PI * 2);
    ctx.fillStyle = bodyGrad;
    ctx.fill();
    ctx.strokeStyle = '#3a3a54';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Body orientation strut (12 o'clock arm)
    ctx.shadowBlur = 0;
    ctx.strokeStyle = '#e4e4f0';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -bodyR * 0.88);
    ctx.stroke();
    // Notch at tip
    ctx.beginPath();
    ctx.arc(0, -bodyR * 0.88, 4, 0, Math.PI * 2);
    ctx.fillStyle = '#e4e4f0';
    ctx.fill();

    // Panel struts (decorative)
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1;
    for (let i = 0; i < 6; i++) {
      const a = (i / 6) * Math.PI * 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(Math.cos(a) * bodyR, Math.sin(a) * bodyR);
      ctx.stroke();
    }

    ctx.restore();

    // --- Reaction wheel (spinning at theta1 — much faster than body) ---
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-(Math.PI / 2) + bodyAngle); // body frame first
    ctx.rotate(snap.trueState.theta1);       // then wheel relative to body

    const wheelGrad = ctx.createRadialGradient(0, 0, hubR, 0, 0, wheelR);
    wheelGrad.addColorStop(0, '#2c2c48');
    wheelGrad.addColorStop(1, '#1e1e32');
    ctx.beginPath();
    ctx.arc(0, 0, wheelR, 0, Math.PI * 2);
    ctx.fillStyle = wheelGrad;
    ctx.fill();
    ctx.strokeStyle = '#5a5a7a';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Wheel spokes
    const spokeCount = 5;
    ctx.strokeStyle = 'rgba(90,90,122,0.5)';
    ctx.lineWidth = 1;
    for (let i = 0; i < spokeCount; i++) {
      const a = (i / spokeCount) * Math.PI * 2;
      ctx.beginPath();
      ctx.moveTo(Math.cos(a) * hubR * 1.5, Math.sin(a) * hubR * 1.5);
      ctx.lineTo(Math.cos(a) * wheelR * 0.9, Math.sin(a) * wheelR * 0.9);
      ctx.stroke();
    }

    // Wheel speed indicator line
    ctx.strokeStyle = '#60a5fa';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -wheelR * 0.8);
    ctx.stroke();

    // Hub
    ctx.beginPath();
    ctx.arc(0, 0, hubR, 0, Math.PI * 2);
    ctx.fillStyle = '#3a3a5a';
    ctx.fill();
    ctx.strokeStyle = '#5a5a7a';
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.restore();

    // --- Estimated angle arm (drawn on top, slightly transparent) ---
    const estAngleCanvas = -(Math.PI / 2) + snap.estimatedAngle;
    ctx.save();
    ctx.globalAlpha = 0.7;
    ctx.strokeStyle = '#34d399';
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(
      cx + Math.cos(estAngleCanvas) * bodyR * 0.95,
      cy + Math.sin(estAngleCanvas) * bodyR * 0.95,
    );
    ctx.stroke();
    ctx.restore();

    // --- Saturation indicator ring ---
    const outputMax = Math.abs(snap.rawControlOutput);
    const saturated = outputMax > Math.abs(snap.controlOutput) * 1.05 + 0.001;
    if (saturated) {
      ctx.save();
      ctx.strokeStyle = 'rgba(248, 113, 113, 0.5)';
      ctx.lineWidth = 3;
      ctx.setLineDash([8, 6]);
      ctx.beginPath();
      ctx.arc(cx, cy, bodyR + 4, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }

    // --- Legend inset ---
    const lx = 10, ly = H - 54;
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.fillStyle = 'rgba(255,255,255,0.35)';

    const legendItems = [
      { color: '#e4e4f0', label: 'True angle' },
      { color: '#34d399', label: 'Estimated' },
      { color: '#7c6eff', label: 'Target' },
      { color: '#fb923c', label: 'Error arc' },
    ];
    legendItems.forEach((item, i) => {
      ctx.fillStyle = item.color;
      ctx.fillRect(lx, ly + i * 13, 7, 7);
      ctx.fillStyle = 'rgba(255,255,255,0.35)';
      ctx.fillText(item.label, lx + 12, ly + i * 13 + 7);
    });
  }
}
