import { ApiError } from "./apiError";

export interface FirmwareCommandResult {
  success: boolean;
  message: string;
}

/** LED pattern ids — see autonav_firmware/firmware/Esp-32-Autonav-4.0/led_pattern.h. */
export const LED_PATTERN_OFF = 0;
export const LED_PATTERN_IDLE = 2;

/** Plain fetch wrapper for RobotServer's /api/firmware/* endpoints (session-gated, same-origin). */
async function request<T>(path: string, body: unknown): Promise<T> {
  const response = await fetch(`/api/firmware${path}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });

  const payload = await response.json().catch(() => undefined);

  if (!response.ok && response.status !== 502) {
    throw new ApiError(payload?.message ?? "Something went wrong", response.status);
  }

  // 502 is a well-formed { success: false, message } from the ESP32 bridge
  // (board disconnected, no ACK, etc.) — a domain result, not a thrown error.
  return payload as T;
}

export const firmwareClient = {
  setLedPattern: (patternId: number) =>
    request<FirmwareCommandResult>("/led/pattern", { patternId }),

  setBrightness: (brightness: number) =>
    request<FirmwareCommandResult>("/led/brightness", { brightness }),
};
