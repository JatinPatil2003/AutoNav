import { WebSocket } from "ws";

import { config } from "../config/env";
import { linkSecretStore } from "../config/linkSecretStore";
import { logger } from "../common/utils/logger.util";
import { DeviceIdentityService } from "../ros/deviceIdentity";
import { IPlatformLink, PlatformLinkFrameHandler } from "./platformLink.interface";

const LINK_PATH = "/ws/robot-link";
const INITIAL_BACKOFF_MS = 1_000;
const MAX_BACKOFF_MS = 30_000;
const PING_TIMEOUT_MS = 25_000;
const PING_WATCHDOG_INTERVAL_MS = PING_TIMEOUT_MS / 2;
const DEVICE_IDENTITY_POLL_MS = 5_000;

/** Minimal surface TeleopService needs to expose for the link's fail-safe teardown — avoids a direct import cycle/coupling to the full service. */
export interface Stoppable {
  stop(): void;
}

/**
 * This robot's outbound half of the Part 2 robot-link: dials
 * PlatformServer's `/ws/robot-link`, authenticates with this robot's id +
 * `ROBOT_LINK_SECRET`, and reconnects forever with jittered exponential
 * backoff. The connection only starts once `deviceIdentity` has resolved
 * a `robotId` (see ros/deviceIdentity.ts) — until then there's no identity
 * to authenticate with, so this polls for it rather than dialing early.
 *
 * Fail-safe is the whole point of this class: on any disconnect (clean
 * close, error, or a missed-ping timeout) it immediately stops the robot
 * via `teleop.stop()` before attempting to reconnect — no link means no
 * remote supervision of motion, so motion must not continue unattended.
 */
export class PlatformLinkClient implements IPlatformLink {
  private socket: WebSocket | undefined;
  private readonly frameHandlers: PlatformLinkFrameHandler[] = [];
  private backoffMs = INITIAL_BACKOFF_MS;
  private lastPingAt = 0;
  private pingWatchdog: NodeJS.Timeout | undefined;
  private reconnectTimer: NodeJS.Timeout | undefined;
  private stopped = true;

  constructor(
    private readonly deviceIdentity: DeviceIdentityService,
    private readonly teleop: Stoppable,
  ) {}

  start(): void {
    if (!this.stopped) return;
    this.stopped = false;
    void this.connectWhenIdentityReady();
  }

  stop(): void {
    this.stopped = true;
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.clearPingWatchdog();
    this.socket?.close();
    this.socket = undefined;
  }

  isConnected(): boolean {
    return this.socket?.readyState === WebSocket.OPEN;
  }

  send(frame: unknown): void {
    if (!this.isConnected()) {
      logger.debug({ frame }, "platformLink: dropping outbound frame, link is down");
      return;
    }
    this.socket!.send(JSON.stringify(frame));
  }

  onFrame(handler: PlatformLinkFrameHandler): void {
    this.frameHandlers.push(handler);
  }

  forceReconnect(): void {
    if (this.stopped) return;
    logger.info("platformLink: forcing reconnect to pick up a rotated credential");

    // Fail-safe, same as any other disconnect — see this class's doc comment.
    this.teleop.stop();

    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.clearPingWatchdog();
    this.backoffMs = INITIAL_BACKOFF_MS;

    const oldSocket = this.socket;
    this.socket = undefined;
    if (oldSocket) {
      // Strip listeners before terminating so the old socket's own
      // close/error handlers don't also fire teardownAndScheduleReconnect()
      // — that would race a second reconnect attempt against the explicit
      // one below.
      oldSocket.removeAllListeners();
      oldSocket.terminate();
    }

    void this.connectWhenIdentityReady();
  }

  private async connectWhenIdentityReady(): Promise<void> {
    let robotId = this.deviceIdentity.getRobotId();
    while (!robotId && !this.stopped) {
      robotId = await this.deviceIdentity.resolve();
      if (!robotId) {
        await delay(DEVICE_IDENTITY_POLL_MS);
      }
    }
    if (this.stopped || !robotId) return;
    this.dial(robotId);
  }

  private dial(robotId: string): void {
    if (this.stopped) return;

    const url = `${deriveWsBaseUrl(config.platform.baseUrl)}${LINK_PATH}`;
    const socket = new WebSocket(url, {
      headers: {
        "x-robot-id": robotId,
        Authorization: `Robot ${linkSecretStore.getSecret()}`,
      },
    });
    this.socket = socket;

    socket.on("open", () => {
      logger.info({ url }, "platformLink: connected to PlatformServer");
      this.backoffMs = INITIAL_BACKOFF_MS;
      this.lastPingAt = Date.now();
      this.startPingWatchdog();
    });

    // `ws` auto-responds to protocol-level pings with a pong by default —
    // no explicit pong handling needed here. We only need to observe that
    // pings keep arriving at all, to detect a silently-dead connection
    // (see the watchdog below) rather than relying solely on a TCP-level
    // close/error that may never fire.
    socket.on("ping", () => {
      this.lastPingAt = Date.now();
    });

    socket.on("message", (raw) => {
      let frame: unknown;
      try {
        frame = JSON.parse(raw.toString());
      } catch (err) {
        logger.warn({ err }, "platformLink: dropping non-JSON frame");
        return;
      }
      for (const handler of this.frameHandlers) handler(frame);
    });

    socket.on("close", (code) => {
      logger.warn({ code }, "platformLink: connection closed");
      this.teardownAndScheduleReconnect();
    });

    socket.on("error", (err) => {
      logger.warn({ err }, "platformLink: connection error");
    });
  }

  private startPingWatchdog(): void {
    this.clearPingWatchdog();
    this.pingWatchdog = setInterval(() => {
      if (Date.now() - this.lastPingAt > PING_TIMEOUT_MS) {
        logger.warn("platformLink: no ping received within timeout, forcing reconnect");
        this.socket?.terminate(); // triggers 'close', which runs the fail-safe + reconnect below
      }
    }, PING_WATCHDOG_INTERVAL_MS).unref();
  }

  private clearPingWatchdog(): void {
    if (this.pingWatchdog) {
      clearInterval(this.pingWatchdog);
      this.pingWatchdog = undefined;
    }
  }

  private teardownAndScheduleReconnect(): void {
    // Fail-safe: no link ⇒ no remote supervision of motion. Non-negotiable,
    // so this runs unconditionally before anything else, including when
    // `stop()` was the cause of this close.
    this.teleop.stop();

    this.clearPingWatchdog();
    this.socket = undefined;

    if (this.stopped) return;

    const jitterFactor = 0.8 + Math.random() * 0.4; // ±20%
    const delayMs = this.backoffMs * jitterFactor;
    this.backoffMs = Math.min(this.backoffMs * 2, MAX_BACKOFF_MS);

    this.reconnectTimer = setTimeout(() => {
      void this.connectWhenIdentityReady();
    }, delayMs).unref();
  }
}

function deriveWsBaseUrl(httpBaseUrl: string): string {
  return httpBaseUrl.replace(/^http/, "ws");
}

function delay(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
