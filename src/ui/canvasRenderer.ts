/**
 * canvasRenderer.ts — Phase 7: Three.js 3D visualization
 *
 * Replaces the Phase 5 2D canvas renderer with a Three.js WebGL scene.
 * The PUBLIC API is identical to the Phase 5 version:
 *   - CanvasRendererOptions interface
 *   - CanvasRenderer class with constructor(options), resize(w,h), render(snap)
 *
 * main.ts requires ZERO changes — it still calls the same methods on the same class.
 *
 * Scene description:
 *   Camera:    Perspective, positioned at (0, 9, 5), looking at origin — slight 3/4 angle
 *              gives depth while keeping the top-down rotational motion clearly readable.
 *   Body:      Flat CylinderGeometry disc (large, dark) rotating around world Y-axis by theta2.
 *   Arm:       Thin box standing out of the body face — shows body pointing direction (white).
 *   Wheel:     Smaller CylinderGeometry (blue-grey) sitting inside body, rotates by theta1 total.
 *   Hub:       Small cylinder at wheel centre.
 *   Spokes:    5 thin boxes radiating from wheel hub.
 *   Target:    Thin vertical plane / line at target angle (purple, semi-transparent).
 *   Error arc: LineLoop arc between body direction and target direction (orange).
 *   Est. arm:  Thin green line showing estimated angle (slightly shorter than true arm).
 *   Sat. ring: Pulsing red torus around body rim when actuator is saturated.
 *   Lighting:  Ambient + two directional lights for metallic depth.
 *
 * R2.3 — reads only from SimSnapshot fields. Zero imports from core/ modules.
 * PRD §8 — Three.js used only here, after core is fully validated (Phase 0-6 complete).
 */

import * as THREE from 'three';

// ---------------------------------------------------------------------------
// Public interface (unchanged from Phase 5)
// ---------------------------------------------------------------------------

export interface CanvasRendererOptions {
  /** Canvas element to draw into. */
  canvas: HTMLCanvasElement;
  /** Physical width [px]. */
  width: number;
  /** Physical height [px]. */
  height: number;
}

// Type for the snapshot subset the renderer uses (same as Phase 5)
type RenderSnap = {
  trueState: { theta1: number; theta2: number; omega1: number; omega2: number };
  estimatedAngle: number;
  setpoint: number;
  controlOutput: number;
  rawControlOutput: number;
};

// ---------------------------------------------------------------------------
// Scene constants
// ---------------------------------------------------------------------------

const BODY_RADIUS    = 2.4;
const BODY_HEIGHT    = 0.22;
const WHEEL_RADIUS   = 0.92;
const WHEEL_HEIGHT   = 0.42;
const HUB_RADIUS     = 0.18;
const HUB_HEIGHT     = 0.52;
const ARM_WIDTH      = 0.09;
const ARM_DEPTH      = 0.09;
const ARM_LENGTH     = BODY_RADIUS * 0.84;
const SPOKE_COUNT    = 5;

// Colours
const C_BODY_TOP    = 0x1e1e30;
const C_BODY_SIDE   = 0x15151f;
const C_WHEEL       = 0x1c2a3a;
const C_HUB         = 0x2e2e44;
const C_ARM         = 0xe8e8f2;
const C_SPOKE       = 0x4a4a6a;
const C_TARGET      = 0x7c6eff;
const C_ERROR_ARC   = 0xfb923c;
const C_EST_ARM     = 0x34d399;
const C_SAT_RING    = 0xf87171;
const C_FLOOR       = 0x0d0d11;

// ---------------------------------------------------------------------------
// CanvasRenderer class (Three.js — same public API as Phase 5)
// ---------------------------------------------------------------------------

export class CanvasRenderer {
  // WebGL renderer
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene:    THREE.Scene;
  private readonly camera:   THREE.PerspectiveCamera;

  // Animated scene objects (refs needed for per-frame update)
  private readonly bodyGroup:   THREE.Group;   // rotates by theta2 around Y
  private readonly wheelGroup:  THREE.Group;   // rotates by theta1 around Y (child of bodyGroup → absolute)
  private readonly targetGroup: THREE.Group;   // rotates by setpoint around Y
  private readonly estArm:      THREE.Line;    // rotates by estimatedAngle around Y
  private readonly errorArc:    THREE.Line;    // rebuilt each frame
  private readonly satRing:     THREE.Mesh;    // visible when saturated
  private          errorArcPoints: THREE.Vector3[] = [];

  // Error arc geometry reference for update
  private readonly errorArcGeom: THREE.BufferGeometry;

  private width:  number;
  private height: number;

  constructor(options: CanvasRendererOptions) {
    this.width  = options.width  || 400;
    this.height = options.height || 400;

    // ── WebGLRenderer ──
    this.renderer = new THREE.WebGLRenderer({
      canvas: options.canvas,
      antialias: true,
      alpha: false,
    });
    this.renderer.setSize(this.width, this.height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x0d0d11, 1);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // ── Camera ──
    this.camera = new THREE.PerspectiveCamera(48, this.width / this.height, 0.1, 100);
    this.camera.position.set(0, 9, 5.5);
    this.camera.lookAt(0, 0, 0);

    // ── Scene ──
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(0x0d0d11, 18, 30);

    // ── Lighting ──
    const ambient = new THREE.AmbientLight(0x303050, 3.5);
    this.scene.add(ambient);

    const dirA = new THREE.DirectionalLight(0xffffff, 2.5);
    dirA.position.set(4, 8, 4);
    dirA.castShadow = true;
    dirA.shadow.mapSize.set(1024, 1024);
    dirA.shadow.camera.near = 0.5;
    dirA.shadow.camera.far  = 30;
    dirA.shadow.camera.left = -6;
    dirA.shadow.camera.right = 6;
    dirA.shadow.camera.top = 6;
    dirA.shadow.camera.bottom = -6;
    this.scene.add(dirA);

    const dirB = new THREE.DirectionalLight(0x6080ff, 1.2);
    dirB.position.set(-5, 4, -3);
    this.scene.add(dirB);

    // ── Floor ──
    const floorGeo  = new THREE.CircleGeometry(10, 64);
    const floorMat  = new THREE.MeshStandardMaterial({ color: C_FLOOR, roughness: 0.9, metalness: 0.1 });
    const floor     = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -(BODY_HEIGHT / 2 + 0.01);
    floor.receiveShadow = true;
    this.scene.add(floor);

    // ── Grid rings (faint, on floor) ──
    for (let r = 1.5; r <= 4.5; r += 1.5) {
      const ringGeo = new THREE.RingGeometry(r - 0.005, r + 0.005, 64);
      const ringMat = new THREE.MeshBasicMaterial({ color: 0x1a1a28, side: THREE.DoubleSide });
      const ring    = new THREE.Mesh(ringGeo, ringMat);
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = -(BODY_HEIGHT / 2 + 0.005);
      this.scene.add(ring);
    }

    // ── Body group (rotates at theta2) ──
    this.bodyGroup = new THREE.Group();
    this.scene.add(this.bodyGroup);

    // Body disc
    const bodyTopMat  = new THREE.MeshStandardMaterial({ color: C_BODY_TOP,  roughness: 0.4, metalness: 0.6 });
    const bodySideMat = new THREE.MeshStandardMaterial({ color: C_BODY_SIDE, roughness: 0.6, metalness: 0.4 });
    const bodyGeo     = new THREE.CylinderGeometry(BODY_RADIUS, BODY_RADIUS, BODY_HEIGHT, 72, 1);

    // Multi-material cylinder (top/bottom = bodyTop, side = bodySide)
    const bodyMesh = new THREE.Mesh(bodyGeo, [bodySideMat, bodyTopMat, bodyTopMat]);
    bodyMesh.castShadow    = true;
    bodyMesh.receiveShadow = true;
    this.bodyGroup.add(bodyMesh);

    // Body rim highlight
    const rimGeo = new THREE.TorusGeometry(BODY_RADIUS, 0.04, 8, 72);
    const rimMat = new THREE.MeshStandardMaterial({ color: 0x3a3a54, roughness: 0.3, metalness: 0.8 });
    const rim    = new THREE.Mesh(rimGeo, rimMat);
    rim.rotation.x = Math.PI / 2;
    rim.position.y = BODY_HEIGHT / 2;
    this.bodyGroup.add(rim);

    // Body struts (6 radial lines on top face)
    const strutMat = new THREE.MeshStandardMaterial({ color: 0x252535, roughness: 0.8 });
    for (let i = 0; i < 6; i++) {
      const angle  = (i / 6) * Math.PI * 2;
      const strGeo = new THREE.BoxGeometry(0.04, 0.01, BODY_RADIUS * 0.9);
      const strut  = new THREE.Mesh(strGeo, strutMat);
      strut.position.set(
        Math.sin(angle) * BODY_RADIUS * 0.45,
        BODY_HEIGHT / 2 + 0.005,
        Math.cos(angle) * BODY_RADIUS * 0.45,
      );
      strut.rotation.y = angle;
      this.bodyGroup.add(strut);
    }

    // Body pointing arm (white rod standing from top face toward +Z → "12 o'clock" direction)
    const armGeo = new THREE.BoxGeometry(ARM_WIDTH, ARM_WIDTH, ARM_LENGTH);
    const armMat = new THREE.MeshStandardMaterial({ color: C_ARM, roughness: 0.3, metalness: 0.5, emissive: 0x404048 });
    const arm    = new THREE.Mesh(armGeo, armMat);
    arm.position.set(0, BODY_HEIGHT / 2 + ARM_WIDTH / 2, -ARM_LENGTH / 2);
    arm.castShadow = true;
    this.bodyGroup.add(arm);

    // Arm tip sphere
    const tipGeo  = new THREE.SphereGeometry(0.1, 12, 8);
    const tipMesh = new THREE.Mesh(tipGeo, armMat);
    tipMesh.position.set(0, BODY_HEIGHT / 2 + ARM_WIDTH / 2, -ARM_LENGTH);
    this.bodyGroup.add(tipMesh);

    // ── Wheel group (child of scene, NOT body — absolute angle = theta1) ──
    this.wheelGroup = new THREE.Group();
    this.scene.add(this.wheelGroup);

    // Wheel disc
    const wheelMat = new THREE.MeshStandardMaterial({ color: C_WHEEL, roughness: 0.3, metalness: 0.8 });
    const wheelGeo = new THREE.CylinderGeometry(WHEEL_RADIUS, WHEEL_RADIUS, WHEEL_HEIGHT, 40, 1);
    const wheelMesh = new THREE.Mesh(wheelGeo, [wheelMat, wheelMat, wheelMat]);
    wheelMesh.castShadow = true;
    this.wheelGroup.add(wheelMesh);

    // Wheel rim
    const wRimGeo = new THREE.TorusGeometry(WHEEL_RADIUS, 0.03, 6, 40);
    const wRimMat = new THREE.MeshStandardMaterial({ color: 0x5060a0, roughness: 0.2, metalness: 0.9 });
    const wRim    = new THREE.Mesh(wRimGeo, wRimMat);
    wRim.rotation.x = Math.PI / 2;
    wRim.position.y = WHEEL_HEIGHT / 2;
    this.wheelGroup.add(wRim);

    // Wheel spokes
    const spokeMat = new THREE.MeshStandardMaterial({ color: C_SPOKE, roughness: 0.6, metalness: 0.4 });
    for (let i = 0; i < SPOKE_COUNT; i++) {
      const angle   = (i / SPOKE_COUNT) * Math.PI * 2;
      const spokLen = WHEEL_RADIUS * 0.85;
      const spkGeo  = new THREE.BoxGeometry(0.06, WHEEL_HEIGHT * 0.85, spokLen);
      const spoke   = new THREE.Mesh(spkGeo, spokeMat);
      spoke.position.set(
        Math.sin(angle) * spokLen * 0.5,
        0,
        Math.cos(angle) * spokLen * 0.5,
      );
      spoke.rotation.y = angle;
      this.wheelGroup.add(spoke);
    }

    // Wheel hub
    const hubGeo  = new THREE.CylinderGeometry(HUB_RADIUS, HUB_RADIUS, HUB_HEIGHT, 16);
    const hubMat  = new THREE.MeshStandardMaterial({ color: C_HUB, roughness: 0.4, metalness: 0.7 });
    const hub     = new THREE.Mesh(hubGeo, hubMat);
    hub.position.y = 0;
    this.wheelGroup.add(hub);

    // Wheel speed indicator stripe (glows on top)
    const stripeGeo = new THREE.BoxGeometry(0.08, 0.01, WHEEL_RADIUS * 0.85);
    const stripeMat = new THREE.MeshStandardMaterial({ color: 0x60a5fa, emissive: 0x1a3a60 });
    const stripe    = new THREE.Mesh(stripeGeo, stripeMat);
    stripe.position.set(0, WHEEL_HEIGHT / 2 + 0.005, -WHEEL_RADIUS * 0.425);
    this.wheelGroup.add(stripe);

    // ── Target angle marker (purple plane) ──
    this.targetGroup = new THREE.Group();
    this.scene.add(this.targetGroup);

    const targetPlaneMat = new THREE.MeshBasicMaterial({
      color: C_TARGET, transparent: true, opacity: 0.25, side: THREE.DoubleSide,
    });
    const targetPlaneGeo = new THREE.PlaneGeometry(0.06, BODY_RADIUS * 1.25);
    const targetPlane    = new THREE.Mesh(targetPlaneGeo, targetPlaneMat);
    targetPlane.rotation.x = Math.PI / 2;
    targetPlane.position.set(0, BODY_HEIGHT / 2 + 0.03, -(BODY_RADIUS * 1.25) / 2);
    this.targetGroup.add(targetPlane);

    // Target dot at tip
    const tDotGeo = new THREE.SphereGeometry(0.1, 10, 8);
    const tDotMat = new THREE.MeshBasicMaterial({ color: C_TARGET });
    const tDot    = new THREE.Mesh(tDotGeo, tDotMat);
    tDot.position.set(0, BODY_HEIGHT / 2 + 0.05, -(BODY_RADIUS * 1.22));
    this.targetGroup.add(tDot);

    // Target dashed line (LineSegments as dashes)
    {
      const pts: THREE.Vector3[] = [];
      const segments = 12;
      for (let i = 0; i < segments; i++) {
        const t0 = (i + 0.1) / segments;
        const t1 = (i + 0.8) / segments;
        pts.push(new THREE.Vector3(0, BODY_HEIGHT / 2 + 0.04, -t0 * BODY_RADIUS * 1.2));
        pts.push(new THREE.Vector3(0, BODY_HEIGHT / 2 + 0.04, -t1 * BODY_RADIUS * 1.2));
      }
      const tLineGeo = new THREE.BufferGeometry().setFromPoints(pts);
      const tLineMat = new THREE.LineBasicMaterial({ color: C_TARGET, transparent: true, opacity: 0.7 });
      this.targetGroup.add(new THREE.LineSegments(tLineGeo, tLineMat));
    }

    // ── Estimated angle arm (green dashed line) ──
    {
      const pts: THREE.Vector3[] = [];
      const segs = 10;
      for (let i = 0; i < segs; i++) {
        const t0 = (i + 0.1) / segs;
        const t1 = (i + 0.8) / segs;
        pts.push(new THREE.Vector3(0, BODY_HEIGHT / 2 + 0.06, -t0 * BODY_RADIUS * 0.92));
        pts.push(new THREE.Vector3(0, BODY_HEIGHT / 2 + 0.06, -t1 * BODY_RADIUS * 0.92));
      }
      const estGeo = new THREE.BufferGeometry().setFromPoints(pts);
      const estMat = new THREE.LineBasicMaterial({ color: C_EST_ARM, transparent: true, opacity: 0.8 });
      this.estArm = new THREE.LineSegments(estGeo, estMat);
      this.scene.add(this.estArm);
    }

    // ── Error arc (orange — built dynamically each frame) ──
    this.errorArcGeom = new THREE.BufferGeometry();
    const errorMat    = new THREE.LineBasicMaterial({ color: C_ERROR_ARC, linewidth: 2 });
    this.errorArc     = new THREE.Line(this.errorArcGeom, errorMat);
    this.errorArc.position.y = BODY_HEIGHT / 2 + 0.08;
    this.scene.add(this.errorArc);

    // ── Saturation ring (red torus around body rim) ──
    const satGeo = new THREE.TorusGeometry(BODY_RADIUS + 0.18, 0.06, 8, 72);
    const satMat = new THREE.MeshBasicMaterial({ color: C_SAT_RING, transparent: true, opacity: 0.55 });
    this.satRing = new THREE.Mesh(satGeo, satMat);
    this.satRing.rotation.x = Math.PI / 2;
    this.satRing.position.y = BODY_HEIGHT / 2;
    this.satRing.visible = false;
    this.scene.add(this.satRing);
  }

  // ---------------------------------------------------------------------------
  // resize — called by ResizeObserver in main.ts (same signature as Phase 5)
  // ---------------------------------------------------------------------------

  resize(width: number, height: number): void {
    this.width  = width;
    this.height = height;
    this.renderer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  // ---------------------------------------------------------------------------
  // render — called once per requestAnimationFrame (same signature as Phase 5)
  // ---------------------------------------------------------------------------

  render(snap: RenderSnap): void {
    const { trueState, estimatedAngle, setpoint, controlOutput, rawControlOutput } = snap;

    // ── 1. Body rotation (theta2 around Y axis) ──
    // In Three.js: Y is up. We rotate in the horizontal (XZ) plane.
    // theta2 = 0 → arm points toward -Z (camera-forward). theta2 increases CCW viewed from above.
    this.bodyGroup.rotation.y = -trueState.theta2;

    // ── 2. Wheel rotation (absolute theta1) ──
    // The wheel is a separate scene object (not child of body) so it rotates
    // at absolute theta1, independent of the body. This correctly shows
    // wheel angle regardless of body rotation — matching PRD §6.1 (theta1 tracked separately).
    this.wheelGroup.rotation.y = -trueState.theta1;

    // ── 3. Target marker ──
    this.targetGroup.rotation.y = -setpoint;

    // ── 4. Estimated angle arm ──
    this.estArm.rotation.y = -estimatedAngle;

    // ── 5. Error arc ──
    this._updateErrorArc(trueState.theta2, setpoint);

    // ── 6. Saturation indicator ──
    const saturated = Math.abs(rawControlOutput) > Math.abs(controlOutput) + 1e-4;
    this.satRing.visible = saturated;
    if (saturated) {
      // Pulse opacity using sin(time)
      const t = performance.now() / 300;
      (this.satRing.material as THREE.MeshBasicMaterial).opacity = 0.35 + 0.2 * Math.sin(t);
    }

    // ── 7. Render ──
    this.renderer.render(this.scene, this.camera);
  }

  // ---------------------------------------------------------------------------
  // Private helpers
  // ---------------------------------------------------------------------------

  /** Rebuild the error arc geometry for the current frame (arc from body to target). */
  private _updateErrorArc(bodyAngle: number, targetAngle: number): void {
    let err = targetAngle - bodyAngle;

    // Keep arc short (< π) — take the short way around
    while (err >  Math.PI) err -= 2 * Math.PI;
    while (err < -Math.PI) err += 2 * Math.PI;

    const arcR   = BODY_RADIUS + 0.38;
    const steps  = Math.max(4, Math.ceil(Math.abs(err) / (Math.PI / 32)));
    const pts: THREE.Vector3[] = [];

    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      const a = bodyAngle + t * err;
      pts.push(new THREE.Vector3(
        Math.sin(-a) * arcR,
        0,
        Math.cos(-a) * arcR,
      ));
    }

    this.errorArcGeom.setFromPoints(pts);
    this.errorArcGeom.computeBoundingSphere();
  }
}
