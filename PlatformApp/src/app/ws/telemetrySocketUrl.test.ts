import { afterEach, describe, expect, it, vi } from "vitest";
import { buildTelemetrySocketUrl } from "./telemetrySocketUrl";

describe("buildTelemetrySocketUrl", () => {
  afterEach(() => {
    vi.unstubAllEnvs();
  });

  it("derives wss:// from an https API base URL and strips the /api suffix", () => {
    vi.stubEnv("VITE_API_BASE_URL", "https://platform-server.autonav.in/api");

    const url = buildTelemetrySocketUrl("abc123");

    expect(url).toBe("wss://platform-server.autonav.in/ws/telemetry?robotId=abc123");
  });

  it("derives ws:// (not wss://) from a plain http API base URL", () => {
    vi.stubEnv("VITE_API_BASE_URL", "http://localhost:4000/api");

    const url = buildTelemetrySocketUrl("abc123");

    expect(url).toBe("ws://localhost:4000/ws/telemetry?robotId=abc123");
  });

  it("url-encodes the robotId query param", () => {
    vi.stubEnv("VITE_API_BASE_URL", "http://localhost:4000/api");

    const url = buildTelemetrySocketUrl("has space");

    expect(url).toBe("ws://localhost:4000/ws/telemetry?robotId=has+space");
  });
});
