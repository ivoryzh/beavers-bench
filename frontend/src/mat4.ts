// Minimal 4x4 matrix helpers (row-major number[16]) so the kinematics has no three.js dependency
// and can be unit tested in node.

export type Mat4 = number[];
export type Vec3 = [number, number, number];

export const identity = (): Mat4 => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

export function multiply(a: Mat4, b: Mat4): Mat4 {
  const out = new Array<number>(16);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      out[r * 4 + c] =
        a[r * 4] * b[c] + a[r * 4 + 1] * b[4 + c] + a[r * 4 + 2] * b[8 + c] + a[r * 4 + 3] * b[12 + c];
    }
  }
  return out;
}

export const chain = (...ms: Mat4[]): Mat4 => ms.reduce((acc, m) => multiply(acc, m), identity());

/** Inverse of a rigid transform (rotation + translation). */
export function invertRigid(m: Mat4): Mat4 {
  const r = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]];
  const t = [m[3], m[7], m[11]];
  // R^T and -R^T t
  const rt = [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]];
  const nt = [
    -(rt[0] * t[0] + rt[1] * t[1] + rt[2] * t[2]),
    -(rt[3] * t[0] + rt[4] * t[1] + rt[5] * t[2]),
    -(rt[6] * t[0] + rt[7] * t[1] + rt[8] * t[2]),
  ];
  return [rt[0], rt[1], rt[2], nt[0], rt[3], rt[4], rt[5], nt[1], rt[6], rt[7], rt[8], nt[2], 0, 0, 0, 1];
}

export const translation = (m: Mat4): Vec3 => [m[3], m[7], m[11]];
export const column = (m: Mat4, c: number): Vec3 => [m[c], m[4 + c], m[8 + c]];

export function rotX(a: number): Mat4 {
  const c = Math.cos(a), s = Math.sin(a);
  return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
}
export function rotY(a: number): Mat4 {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
}
export function rotZ(a: number): Mat4 {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}
export const translate = (x: number, y: number, z: number): Mat4 => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];

/** URDF-style pose: fixed-axis roll/pitch/yaw (R = Rz(yaw) Ry(pitch) Rx(roll)) then translation. */
export function rpyXyz(roll: number, pitch: number, yaw: number, x: number, y: number, z: number): Mat4 {
  return chain(translate(x, y, z), rotZ(yaw), rotY(pitch), rotX(roll));
}

/** Rotation matrix from a UR axis-angle vector (rx, ry, rz), the orientation a teach pendant shows. */
export function fromAxisAngle(rx: number, ry: number, rz: number): Mat4 {
  const theta = Math.hypot(rx, ry, rz);
  if (theta < 1e-12) return identity();
  const kx = rx / theta, ky = ry / theta, kz = rz / theta;
  const c = Math.cos(theta), s = Math.sin(theta), v = 1 - c;
  return [
    kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s, 0,
    kx * ky * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s, 0,
    kx * kz * v - ky * s, ky * kz * v + kx * s, kz * kz * v + c, 0,
    0, 0, 0, 1,
  ];
}

/** UR axis-angle vector of a rotation matrix (inverse of fromAxisAngle). */
export function toAxisAngle(m: Mat4): Vec3 {
  const trace = m[0] + m[5] + m[10];
  const c = Math.min(1, Math.max(-1, (trace - 1) / 2));
  const theta = Math.acos(c);
  if (theta < 1e-9) return [0, 0, 0];
  if (Math.PI - theta < 1e-6) {
    // 180 degrees: axis from the symmetric part
    const kx = Math.sqrt(Math.max(0, (m[0] + 1) / 2));
    const ky = Math.sqrt(Math.max(0, (m[5] + 1) / 2)) * (m[1] < 0 ? -1 : 1);
    const kz = Math.sqrt(Math.max(0, (m[10] + 1) / 2)) * (m[2] < 0 ? -1 : 1);
    return [kx * theta, ky * theta, kz * theta];
  }
  const f = theta / (2 * Math.sin(theta));
  return [(m[9] - m[6]) * f, (m[2] - m[8]) * f, (m[4] - m[1]) * f];
}

/** Pose [x, y, z, rx, ry, rz] (metres, UR axis-angle) to a matrix. */
export function fromPose(p: number[]): Mat4 {
  const r = fromAxisAngle(p[3], p[4], p[5]);
  r[3] = p[0];
  r[7] = p[1];
  r[11] = p[2];
  return r;
}

export function toPose(m: Mat4): number[] {
  const [rx, ry, rz] = toAxisAngle(m);
  return [m[3], m[7], m[11], rx, ry, rz];
}

/** Linear interpolation of a rigid transform: position lerp, rotation slerp via axis-angle of the delta. */
export function interpolate(a: Mat4, b: Mat4, t: number): Mat4 {
  const ra = [...a]; ra[3] = 0; ra[7] = 0; ra[11] = 0;
  const rb = [...b]; rb[3] = 0; rb[7] = 0; rb[11] = 0;
  const delta = multiply(invertRigid(ra), rb); // a^-1 b
  const [dx, dy, dz] = toAxisAngle(delta);
  const r = multiply(ra, fromAxisAngle(dx * t, dy * t, dz * t));
  r[3] = a[3] + (b[3] - a[3]) * t;
  r[7] = a[7] + (b[7] - a[7]) * t;
  r[11] = a[11] + (b[11] - a[11]) * t;
  return r;
}

export function approxEqual(a: Mat4, b: Mat4, tol = 1e-6): boolean {
  return a.every((v, i) => Math.abs(v - b[i]) < tol);
}
