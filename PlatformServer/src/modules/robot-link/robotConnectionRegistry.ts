import { randomUUID } from "node:crypto";
import { EventEmitter } from "node:events";
import { WebSocket } from "ws";
import { logger } from "../../common/utils/logger.util";
import { IRobotRepository } from "../robots/robot.repository.interface";
import { IRobotConnectionRegistry } from "./robotConnectionRegistry.interface";
import { healthQueryFrameSchema, HealthResult, RobotLinkOutboundFrame } from "./robotLink.protocol";

const PING_INTERVAL_MS = 10_000;
const MAX_MISSED_PONGS = 2;
/** Close code for "a newer connection for this robotId just replaced you" — mirrors the HTTP 409 convention. */
const REPLACED_CLOSE_CODE = 4409;

interface Entry {
  socket: WebSocket;
  missedPongs: number;
  lastRttMs?: number;
  lastPingAt?: number;
  connectedAt: Date;
  pingTimer: ReturnType<typeof setInterval>;
  pendingHealthQueries: Map<string, (result: HealthResult | null) => void>;
}

/**
 * Tracks the one persistent `/ws/robot-link` connection per robot. A newer
 * connection for the same robotId always wins — the older socket is closed
 * with 4409 rather than left to linger and race the new one. Liveness is
 * enforced server-side with a ping/pong watchdog rather than trusting the
 * robot to ever say goodbye cleanly (flaky VPN links won't always get a
 * close frame out).
 */
export class RobotConnectionRegistry extends EventEmitter implements IRobotConnectionRegistry {
  private readonly entries = new Map<string, Entry>();

  constructor(private readonly robotRepository: IRobotRepository) {
    super();
  }

  register(robotId: string, socket: WebSocket): void {
    const existing = this.entries.get(robotId);
    if (existing) {
      clearInterval(existing.pingTimer);
      existing.socket.close(REPLACED_CLOSE_CODE, "replaced by a newer connection");
    }

    const entry: Entry = {
      socket,
      missedPongs: 0,
      connectedAt: new Date(),
      pendingHealthQueries: new Map(),
      pingTimer: setInterval(() => this.pingOnce(robotId), PING_INTERVAL_MS),
    };
    this.entries.set(robotId, entry);

    socket.on("pong", () => {
      const current = this.entries.get(robotId);
      if (!current || current.socket !== socket) return;
      current.missedPongs = 0;
      if (current.lastPingAt !== undefined) {
        current.lastRttMs = Date.now() - current.lastPingAt;
      }
    });

    void this.robotRepository.touchLastSeen(robotId).catch((err) => {
      logger.warn({ err, robotId }, "RobotConnectionRegistry: failed to touch lastSeenAt on connect");
    });
  }

  unregister(robotId: string, socket: WebSocket): void {
    const entry = this.entries.get(robotId);
    if (!entry || entry.socket !== socket) {
      // Already replaced by a newer connection — that registration owns the cleanup/emit.
      return;
    }

    clearInterval(entry.pingTimer);
    for (const resolve of entry.pendingHealthQueries.values()) {
      resolve(null);
    }
    this.entries.delete(robotId);

    void this.robotRepository.touchLastSeen(robotId).catch((err) => {
      logger.warn({ err, robotId }, "RobotConnectionRegistry: failed to touch lastSeenAt on disconnect");
    });

    this.emit("disconnect", robotId);
  }

  get(robotId: string): WebSocket | undefined {
    return this.entries.get(robotId)?.socket;
  }

  isOnline(robotId: string): boolean {
    const entry = this.entries.get(robotId);
    return !!entry && entry.socket.readyState === WebSocket.OPEN;
  }

  getRttMs(robotId: string): number | undefined {
    return this.entries.get(robotId)?.lastRttMs;
  }

  getConnectedSince(robotId: string): Date | undefined {
    return this.entries.get(robotId)?.connectedAt;
  }

  send(robotId: string, frame: RobotLinkOutboundFrame): boolean {
    const entry = this.entries.get(robotId);
    if (!entry || entry.socket.readyState !== WebSocket.OPEN) return false;
    entry.socket.send(JSON.stringify(frame));
    return true;
  }

  async queryHealth(robotId: string, timeoutMs: number): Promise<HealthResult | null> {
    const entry = this.entries.get(robotId);
    if (!entry || entry.socket.readyState !== WebSocket.OPEN) return null;

    const id = randomUUID();
    const frame = healthQueryFrameSchema.parse({ type: "health.query", id });

    return new Promise((resolve) => {
      const timer = setTimeout(() => {
        entry.pendingHealthQueries.delete(id);
        resolve(null);
      }, timeoutMs);

      entry.pendingHealthQueries.set(id, (result) => {
        clearTimeout(timer);
        resolve(result);
      });

      entry.socket.send(JSON.stringify(frame));
    });
  }

  /** Routes a `health.result` frame received on the link back to whoever is awaiting it, if anyone. */
  resolveHealthResult(robotId: string, id: string, result: HealthResult): void {
    const entry = this.entries.get(robotId);
    const resolve = entry?.pendingHealthQueries.get(id);
    if (!entry || !resolve) return;
    entry.pendingHealthQueries.delete(id);
    resolve(result);
  }

  private pingOnce(robotId: string): void {
    const entry = this.entries.get(robotId);
    if (!entry) return;

    if (entry.socket.readyState !== WebSocket.OPEN) {
      this.unregister(robotId, entry.socket);
      return;
    }

    if (entry.missedPongs >= MAX_MISSED_PONGS) {
      entry.socket.terminate();
      this.unregister(robotId, entry.socket);
      return;
    }

    entry.missedPongs += 1;
    entry.lastPingAt = Date.now();
    entry.socket.ping();
  }
}
