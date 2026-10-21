import { IRobotDocument } from "../robots/robot.model";

/**
 * A thin, generic client for calling a robot's own RobotServer REST API
 * directly over the VPN by vpnHostname — the same network path
 * HttpRobotNotifier already proves works for its one-off roster-invalidate
 * push, generalized so RobotProxyService can call any RobotServer route.
 */
export interface IRobotApiClient {
  request<TResponse>(robot: IRobotDocument, method: "GET" | "POST", path: string, body?: unknown): Promise<TResponse>;
}
