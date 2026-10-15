import { Server as HttpServer } from "node:http";
import { WebSocket, WebSocketServer } from "ws";

import { IncomingMessage } from "node:http";

type WsConnectionHandler = (ws: WebSocket) => void;
type WsAuthenticator = (request: IncomingMessage) => boolean;

/**
 * Routes WebSocket upgrades on one HTTP server to different handlers by
 * pathname (`ws` doesn't do this natively — a WebSocketServer normally
 * owns one path). Each module registers its own path instead of a single
 * god-handler branching on URL, mirroring how Express routers are kept
 * per-module.
 */
export class WsRouter {
  private readonly servers = new Map<string, WebSocketServer>();

  register(path: string, onConnection: WsConnectionHandler): void {
    const wss = new WebSocketServer({ noServer: true });
    wss.on("connection", onConnection);
    this.servers.set(path, wss);
  }

  /**
   * `authenticate` is optional so this stays a generic routing primitive
   * free of any JWT/cookie import — the actual check (RobotApp's session
   * cookie, since it's served same-origin and the browser attaches it to
   * the upgrade request automatically) is injected from app.ts.
   */
  attach(server: HttpServer, authenticate?: WsAuthenticator): void {
    server.on("upgrade", (request, socket, head) => {
      const pathname = new URL(request.url ?? "", "http://localhost").pathname;
      const wss = this.servers.get(pathname);
      if (!wss) {
        socket.destroy();
        return;
      }

      if (authenticate && !authenticate(request)) {
        socket.write("HTTP/1.1 401 Unauthorized\r\n\r\n");
        socket.destroy();
        return;
      }

      wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit("connection", ws, request);
      });
    });
  }
}

export const wsRouter = new WsRouter();
