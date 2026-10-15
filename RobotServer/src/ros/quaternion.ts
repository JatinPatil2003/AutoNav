/**
 * Yaw-only quaternion (roll = pitch = 0), which is all every pose in this
 * codebase ever needs. Avoids pulling in a full tf-transformations
 * dependency just to compute this.
 */
export function quaternionFromYaw(theta: number): { z: number; w: number } {
  return { z: Math.sin(theta / 2), w: Math.cos(theta / 2) };
}

export function yawFromQuaternion(q: { x: number; y: number; z: number; w: number }): number {
  const sinYawCosPitch = 2 * (q.w * q.z + q.x * q.y);
  const cosYawCosPitch = 1 - 2 * (q.y * q.y + q.z * q.z);
  return Math.atan2(sinYawCosPitch, cosYawCosPitch);
}
