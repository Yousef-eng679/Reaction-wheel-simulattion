/**
 * constants.ts
 *
 * Named physical constants for the reaction wheel simulation.
 * All values in SI units: kilograms, meters, radians, rad/s, N·m, seconds (R1.3, R1.4).
 *
 * Every constant is annotated with:
 *   - Its unit
 *   - Whether it is a "measured value" or a "representative placeholder pending real hardware characterization"
 *
 * ⚠️  IMPORTANT: Most values below are PLACEHOLDER ESTIMATES.
 * They are representative of a repurposed HDD spindle motor system but have NOT been
 * measured from the actual hardware. They must be replaced with measured values during
 * the physical-build phase. See PRD §10 and README.
 */

// ---------------------------------------------------------------------------
// Inertia parameters
// ---------------------------------------------------------------------------

/**
 * Default moment of inertia of the reaction wheel (rotor) [kg·m²].
 * PLACEHOLDER — representative estimate for a typical 3.5" HDD rotor.
 * Must be replaced with measured value from hardware characterization.
 */
export const DEFAULT_I1_WHEEL_KGM2: number = 1e-4;

/**
 * Default moment of inertia of the spacecraft body [kg·m²].
 * PLACEHOLDER — representative estimate for a small demo platform (~200g, ~0.1m radius).
 * Must be replaced with measured value from hardware characterization.
 */
export const DEFAULT_I2_BODY_KGM2: number = 5e-3;

// ---------------------------------------------------------------------------
// Motor / actuator parameters
// ---------------------------------------------------------------------------

/**
 * Default maximum motor RPM [rev/min].
 * PLACEHOLDER — representative of typical HDD spindle speed (5400–7200 RPM range).
 * Must be replaced with measured value from hardware characterization.
 */
export const DEFAULT_MOTOR_MAX_RPM: number = 5400;

/**
 * Default maximum motor torque [N·m].
 * PLACEHOLDER — representative estimate for a small BLDC/spindle motor.
 * Must be replaced with measured value from hardware characterization.
 */
export const DEFAULT_MOTOR_MAX_TORQUE_NM: number = 0.05;

/**
 * Default motor first-order lag time constant [s].
 * Models the mechanical + electrical response delay of the motor.
 * PLACEHOLDER — representative estimate; real value requires step-response characterization.
 */
export const DEFAULT_MOTOR_TIME_CONSTANT_S: number = 0.1;

// ---------------------------------------------------------------------------
// Friction parameters
// ---------------------------------------------------------------------------

/**
 * Default bearing/air friction coefficient for the wheel [N·m·s/rad].
 * Near-zero — friction is intentionally small to demonstrate near-ideal conservation.
 * PLACEHOLDER — representative estimate; tune based on measured spin-down curves.
 */
export const DEFAULT_FRICTION_COEFF_1: number = 1e-5;

/**
 * Default bearing/air friction coefficient for the spacecraft body [N·m·s/rad].
 * Near-zero — body is assumed to have minimal friction in the demo rig.
 * PLACEHOLDER — representative estimate.
 */
export const DEFAULT_FRICTION_COEFF_2: number = 1e-5;

// ---------------------------------------------------------------------------
// Sensor / estimator parameters
// ---------------------------------------------------------------------------

/**
 * Default gyroscope noise standard deviation [rad/s].
 * PLACEHOLDER — representative of MPU6050 gyro noise density (~0.005 rad/s at 100 Hz).
 * Must be replaced with measured value from sensor characterization.
 */
export const DEFAULT_GYRO_NOISE_SIGMA_RAD_S: number = 0.005;

/**
 * Default accelerometer-derived angle noise standard deviation [rad].
 * PLACEHOLDER — representative of MPU6050 accelerometer noise + tilt-angle conversion.
 * Must be replaced with measured value from sensor characterization.
 */
export const DEFAULT_ACCEL_ANGLE_NOISE_SIGMA_RAD: number = 0.02;

/**
 * Default gyroscope bias drift rate [rad/s per second].
 * Models the slow random walk of gyroscope bias over time (in-run bias instability).
 * PLACEHOLDER — representative of MPU6050 bias stability.
 */
export const DEFAULT_GYRO_BIAS_DRIFT_RATE: number = 1e-4;

/**
 * Default sensor sample rate [Hz].
 * Can be set slower than the physics rate to demonstrate aliasing/discretization.
 * PLACEHOLDER — MPU6050 can run 100–1000 Hz; 200 Hz is a reasonable default.
 */
export const DEFAULT_SENSOR_SAMPLE_RATE_HZ: number = 200;

// ---------------------------------------------------------------------------
// Complementary filter parameters
// ---------------------------------------------------------------------------

/**
 * Default complementary filter blend coefficient (α) [dimensionless, 0–1].
 * Controls the blend between gyro integration (short-term) and accel (long-term):
 *   estimated_angle = α * (prev_estimate + gyro * dt) + (1 - α) * accel_angle
 * Higher α → trusts gyro more (faster response, more drift).
 * PLACEHOLDER — tunable; 0.98 is a common starting point.
 */
export const DEFAULT_COMPLEMENTARY_ALPHA: number = 0.98;

// ---------------------------------------------------------------------------
// Simulation loop parameters
// ---------------------------------------------------------------------------

/**
 * Internal physics integration timestep [s].
 * Physics always steps at this fixed rate regardless of render framerate (R2.5, PRD §6.1).
 * 1000 Hz (dt = 0.001 s) ensures RK4 accuracy well within conservation tolerance.
 */
export const PHYSICS_DT_S: number = 1e-3;

/**
 * Default PID controller execution rate [Hz].
 * Controller runs at this rate, independent of render framerate (PRD §6.4).
 * Typically matches or is slower than the sensor sample rate.
 */
export const DEFAULT_CONTROLLER_RATE_HZ: number = 200;

// ---------------------------------------------------------------------------
// PID default gains
// ---------------------------------------------------------------------------

/**
 * Default proportional gain Kp [N·m/rad].
 * PLACEHOLDER — tuned empirically for representative inertia values above.
 * Will need retuning with real hardware parameters.
 */
export const DEFAULT_KP: number = 0.5;

/**
 * Default integral gain Ki [N·m/(rad·s)].
 * PLACEHOLDER — tuned empirically; small value to avoid windup issues initially.
 */
export const DEFAULT_KI: number = 0.05;

/**
 * Default derivative gain Kd [N·m·s/rad].
 * PLACEHOLDER — tuned empirically; derivative-on-measurement applied (see pidController.ts).
 */
export const DEFAULT_KD: number = 0.1;

// ---------------------------------------------------------------------------
// Conversion helpers (UI display boundary ONLY — never use inside core/ modules)
// ---------------------------------------------------------------------------

/** Conversion factor: radians per second → RPM. For display use only. */
export const RAD_S_TO_RPM: number = 60 / (2 * Math.PI);

/** Conversion factor: radians → degrees. For display use only. */
export const RAD_TO_DEG: number = 180 / Math.PI;
