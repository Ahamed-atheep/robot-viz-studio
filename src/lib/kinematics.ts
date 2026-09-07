/**
 * Modular kinematics for a 3-DOF articulated (RRR) robotic arm.
 * Configuration is expressed with Denavit–Hartenberg parameters so the
 * robot geometry can be swapped without touching the UI.
 */

export type Mat4 = number[][]; // 4x4, row-major

export interface DHRow {
  /** joint offset along previous z */
  d: number;
  /** link length along x */
  a: number;
  /** link twist about x (radians) */
  alpha: number;
  /** constant added to the joint variable (radians) */
  thetaOffset: number;
}

export interface RobotConfig {
  name: string;
  /** base height */
  L1: number;
  /** upper arm */
  L2: number;
  /** forearm */
  L3: number;
  limits: { min: number; max: number }[]; // degrees
}

export const DEFAULT_ROBOT: RobotConfig = {
  name: "RX-3 Articulated Arm",
  L1: 1.2,
  L2: 1.8,
  L3: 1.4,
  limits: [
    { min: -180, max: 180 },
    { min: -90, max: 150 },
    { min: -150, max: 150 },
  ],
};

export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;

export function dhTable(cfg: RobotConfig): DHRow[] {
  return [
    { d: cfg.L1, a: 0, alpha: Math.PI / 2, thetaOffset: 0 },
    { d: 0, a: cfg.L2, alpha: 0, thetaOffset: 0 },
    { d: 0, a: cfg.L3, alpha: 0, thetaOffset: 0 },
  ];
}

export function identity(): Mat4 {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

export function multiply(A: Mat4, B: Mat4): Mat4 {
  const C = identity();
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
  return C;
}

export function dhMatrix(row: DHRow, theta: number): Mat4 {
  const t = theta + row.thetaOffset;
  const ct = Math.cos(t);
  const st = Math.sin(t);
  const ca = Math.cos(row.alpha);
  const sa = Math.sin(row.alpha);
  return [
    [ct, -st * ca, st * sa, row.a * ct],
    [st, ct * ca, -ct * sa, row.a * st],
    [0, sa, ca, row.d],
    [0, 0, 0, 1],
  ];
}

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface FKResult {
  /** world position of each frame origin, starting at the base (0,0,0) */
  joints: Vec3[];
  endEffector: Vec3;
  /** full 0->3 homogeneous transform */
  transform: Mat4;
}

/** Forward kinematics from joint angles in DEGREES. */
export function forwardKinematics(anglesDeg: number[], cfg: RobotConfig = DEFAULT_ROBOT): FKResult {
  const rows = dhTable(cfg);
  let T = identity();
  const joints: Vec3[] = [{ x: 0, y: 0, z: 0 }];
  rows.forEach((row, i) => {
    T = multiply(T, dhMatrix(row, (anglesDeg[i] ?? 0) * DEG));
    joints.push({ x: T[0][3], y: T[1][3], z: T[2][3] });
  });
  return { joints, endEffector: joints[joints.length - 1], transform: T };
}

export interface IKResult {
  angles: number[]; // degrees
  reachable: boolean;
  withinLimits: boolean;
  reason?: string;
  achieved: Vec3;
  error: number;
}

/** Analytic inverse kinematics (elbow-up) for the RRR arm. */
export function inverseKinematics(
  target: Vec3,
  cfg: RobotConfig = DEFAULT_ROBOT,
  elbowUp = true,
): IKResult {
  const { L1, L2, L3 } = cfg;
  const theta1 = Math.atan2(target.y, target.x);
  const r = Math.hypot(target.x, target.y);
  const s = target.z - L1;
  const dist = Math.hypot(r, s);

  let reachable = true;
  let reason: string | undefined;
  if (dist > L2 + L3 + 1e-9) {
    reachable = false;
    reason = "Target beyond maximum reach";
  } else if (dist < Math.abs(L2 - L3) - 1e-9) {
    reachable = false;
    reason = "Target inside inner dead zone";
  }

  const cos3 = clamp((dist * dist - L2 * L2 - L3 * L3) / (2 * L2 * L3), -1, 1);
  const theta3 = elbowUp ? -Math.acos(cos3) : Math.acos(cos3);
  const theta2 =
    Math.atan2(s, r) - Math.atan2(L3 * Math.sin(theta3), L2 + L3 * Math.cos(theta3));

  const angles = [theta1 * RAD, theta2 * RAD, theta3 * RAD].map(normalizeDeg);
  const fk = forwardKinematics(angles, cfg);
  const error = distance(fk.endEffector, target);

  const withinLimits = angles.every(
    (a, i) => a >= cfg.limits[i].min - 1e-6 && a <= cfg.limits[i].max + 1e-6,
  );
  if (reachable && !withinLimits) reason = "Solution violates joint limits";

  return { angles, reachable, withinLimits, reason, achieved: fk.endEffector, error };
}

export function clamp(v: number, min: number, max: number) {
  return Math.min(max, Math.max(min, v));
}

export function normalizeDeg(a: number) {
  let x = a;
  while (x > 180) x -= 360;
  while (x < -180) x += 360;
  return Math.abs(x) < 1e-9 ? 0 : x;
}

export function distance(a: Vec3, b: Vec3) {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

export function fmt(n: number, digits = 3) {
  return (Object.is(n, -0) ? 0 : n).toFixed(digits);
}

export const WORKSPACE = (cfg: RobotConfig = DEFAULT_ROBOT) => ({
  maxReach: cfg.L2 + cfg.L3,
  minReach: Math.abs(cfg.L2 - cfg.L3),
});
