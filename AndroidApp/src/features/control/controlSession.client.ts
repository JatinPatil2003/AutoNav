import { ClientFrame, ServerFrame, serverFrameSchema } from "./controlSession.protocol";

export type ControlConnectionStatus = "idle" | "connecting" | "open" | "reconnecting" | "closed";

export interface ControlSessionClientOptions {
  wsBaseUrl: string;
  robotId: string;
  /** Called fresh on every (re)connect attempt — the access token can rotate mid-session via baseApi's own refresh flow. */
  getAccessToken: () => string | null;
  onStatusChange: (status: ControlConnectionStatus) => void;
  onServerFrame: (frame: ServerFrame) => void;
}

const RECONNECT_DELAY_MS = 2000;
const MAX_RECONNECT_ATTEMPTS = 5;

/**
 * Thin class+hook WebSocket wrapper for PlatformServer's `/ws/control`
 * relay, styled after RobotServer's reconnect-on-close WS clients
 * (fixed-backoff reconnect) adapted for a browser-engine WebSocket on the
 * client side. Auth rides the `Sec-WebSocket-Protocol` subprotocol
 * (`new WebSocket(url, ["bearer", token])`, per Part 4.5 of the backend
 * plan) since a WS handshake can't carry a custom Authorization header and
 * the token must never go in the query string.
 */
export class ControlSessionClient {
  private ws: WebSocket | null = null;
  private reconnectAttempts = 0;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  // A terminal outcome (explicit close(), or a non-"ready" session frame
  // from the server) must not trigger a reconnect — only an unexpected
  // network drop should.
  private terminal = false;

  constructor(private readonly options: ControlSessionClientOptions) {}

  connect(): void {
    this.terminal = false;
    this.openSocket();
  }

  private openSocket(): void {
    const token = this.options.getAccessToken();
    if (!token) {
      this.terminal = true;
      this.options.onStatusChange("closed");
      return;
    }

    this.options.onStatusChange(this.reconnectAttempts > 0 ? "reconnecting" : "connecting");

    const url = `${this.options.wsBaseUrl}/ws/control?robotId=${encodeURIComponent(this.options.robotId)}`;
    const ws = new WebSocket(url, ["bearer", token]);
    this.ws = ws;

    ws.onopen = () => {
      this.reconnectAttempts = 0;
      this.options.onStatusChange("open");
    };

    ws.onmessage = (event) => {
      let parsed: unknown;
      try {
        parsed = JSON.parse(String(event.data));
      } catch {
        return;
      }

      const result = serverFrameSchema.safeParse(parsed);
      if (!result.success) return;

      if (result.data.type === "session" && result.data.state !== "ready") {
        // preempted/revoked/robot_offline are deliberate server decisions,
        // not transient network blips — reconnecting immediately would
        // just re-hit the same outcome.
        this.terminal = true;
      }

      this.options.onServerFrame(result.data);
    };

    ws.onclose = () => {
      this.ws = null;
      if (this.terminal || this.reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
        this.options.onStatusChange("closed");
        return;
      }

      this.reconnectAttempts += 1;
      this.options.onStatusChange("reconnecting");
      this.reconnectTimer = setTimeout(() => this.openSocket(), RECONNECT_DELAY_MS);
    };

    ws.onerror = () => {
      // A WebSocket error is always followed by a close event — all
      // recovery/backoff logic lives in onclose to avoid double-handling.
    };
  }

  send(frame: ClientFrame): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(frame));
    }
  }

  /** Fire-and-forget e-stop used by the background-safety listeners — bypasses React state entirely. */
  sendEstopImmediate(): void {
    this.send({ type: "estop", data: { engaged: true } });
  }

  close(): void {
    this.terminal = true;
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    this.ws?.close();
    this.ws = null;
    this.options.onStatusChange("closed");
  }
}
