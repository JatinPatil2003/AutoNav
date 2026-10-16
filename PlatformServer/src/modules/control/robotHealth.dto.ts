export interface RobotHealthDto {
  robotId: string;
  healthy: boolean;
  checkedAt: string;
  checks: {
    link: { ok: boolean; connectedSince?: string; rttMs?: number };
    ros: { ok: boolean; navigationActive?: boolean; maxLinear?: number; maxAngular?: number };
  };
}
