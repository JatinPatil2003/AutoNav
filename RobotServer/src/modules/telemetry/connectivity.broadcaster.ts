import { config } from "../../config/env";
import { eventBus } from "../../core/eventBus";
import { logger } from "../../common/utils/logger.util";

const CHECK_PERIOD_MS = 15_000;
const CHECK_TIMEOUT_MS = 5_000;

/**
 * Periodically probes PlatformServer's own /health over the VPN as this
 * robot's proxy for "do we have internet/platform connectivity right
 * now" — the same reachability the roster sync and pin-setup OTP relay
 * already depend on. Only emits onto core/eventBus when the state
 * actually flips, so the "robotStatus" topic isn't noisy at idle.
 */
export function registerConnectivityBroadcast(): NodeJS.Timeout {
  let lastKnown: boolean | null = null;

  const check = async (): Promise<void> => {
    const connected = await probePlatform();
    if (connected !== lastKnown) {
      lastKnown = connected;
      eventBus.emit("robotStatus", { internetConnectivity: connected });
      logger.info({ internetConnectivity: connected }, "connectivity: state changed");
    }
  };

  check();
  return setInterval(check, CHECK_PERIOD_MS).unref();
}

async function probePlatform(): Promise<boolean> {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), CHECK_TIMEOUT_MS);

  try {
    const response = await fetch(`${config.platform.baseUrl}/health`, { signal: controller.signal });
    return response.ok;
  } catch {
    return false;
  } finally {
    clearTimeout(timeout);
  }
}
