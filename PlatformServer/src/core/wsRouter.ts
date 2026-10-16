import { IncomingMessage, Server as HttpServer } from "node:http";
import { Duplex } from "node:stream";
import { WebSocket, WebSocketServer } from "ws";

export type WsConnectionHandler = (ws: WebSocket, request: IncomingMessage) => void;

/**
 * Per-path authenticator: returns a truthy context object to allow the
 * upgrade through (stashed on the request for the connection handler to
 * read back off), or `false`/throws to reject it. Unlike RobotServer's
 * single-authenticator WsRouter, PlatformServer multiplexes two WS
 * endpoints with entirely different trust models on one HTTP listener —
 * `/ws/robot-link` (robot secret header) and `/ws/control` (user JWT +
 * assignment check) — so the authenticator is registered per path rather
 * than once globally.
 */
/**
 * Returns the resolved auth context (e.g. `{ robot }` or `{ user, robotId }`)
 * to allow the upgrade through, or `undefined`/throws to reject it (401).
 * The resolved context is attached to the request as `wsAuthContext` so the
 * connection handler never has to re-derive it.
 */
export type WsAuthenticator<TContext = unknown> = (
  request: IncomingMessage,
) => TContext | undefined | Promise<TContext | undefined>;

declare module "node:http" {
  interface IncomingMessage {
    wsAuthContext?: unknown;
  }
}

interface Registration {
  wss: WebSocketServer;
  authenticate: WsAuthenticator;
}

/**
 * Routes WebSocket upgrades on one HTTP server to different handlers by
 * pathname (`ws` doesn't do this natively — a WebSocketServer normally owns
 * one path), each with its own authenticator run before the upgrade is
 * allowed to complete — so there's never a connected-but-unauthenticated
 * socket to clean up.
 */
export class WsRouter {
  private readonly registrations = new Map<string, Registration>();

  register(path: string, onConnection: WsConnectionHandler, authenticate: WsAuthenticator): void {
    const wss = new WebSocketServer({ noServer: true });
    wss.on("connection", onConnection);
    this.registrations.set(path, { wss, authenticate });
  }

  attach(server: HttpServer): void {
    server.on("upgrade", (request, socket, head) => {
      void this.handleUpgrade(request, socket, head);
    });
  }

  private async handleUpgrade(request: IncomingMessage, socket: Duplex, head: Buffer): Promise<void> {
    const pathname = new URL(request.url ?? "", "http://localhost").pathname;
    const registration = this.registrations.get(pathname);
    if (!registration) {
      socket.destroy();
      return;
    }

    let context: unknown;
    try {
      context = await registration.authenticate(request);
    } catch {
      context = undefined;
    }

    if (!context) {
      socket.write("HTTP/1.1 401 Unauthorized\r\n\r\n");
      socket.destroy();
      return;
    }

    request.wsAuthContext = context;

    registration.wss.handleUpgrade(request, socket, head, (ws) => {
      registration.wss.emit("connection", ws, request);
    });
  }
}

export const wsRouter = new WsRouter();
export type { WebSocket };
