/**
 * VITE_API_BASE_URL is an absolute http(s) URL ending in "/api" (see
 * baseApi.ts). PlatformServer's WS endpoints hang off the same host, not
 * under /api — this derives .../ws/telemetry from it rather than adding a
 * second env var that would always have to be kept in sync with the first.
 */
export function buildTelemetrySocketUrl(robotId: string): string {
  const apiBaseUrl = new URL(import.meta.env.VITE_API_BASE_URL);
  const wsProtocol = apiBaseUrl.protocol === "https:" ? "wss:" : "ws:";
  const basePath = apiBaseUrl.pathname.replace(/\/api\/?$/, "");

  const wsUrl = new URL(`${basePath}/ws/telemetry`, apiBaseUrl.origin);
  wsUrl.protocol = wsProtocol;
  wsUrl.searchParams.set("robotId", robotId);
  return wsUrl.toString();
}
