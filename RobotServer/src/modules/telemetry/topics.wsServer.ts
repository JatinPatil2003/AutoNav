import { WebSocket } from "ws";
import { eventBus } from "../../core/eventBus";
import { WsRouter } from "../../core/wsRouter";
import { logger } from "../../common/utils/logger.util";

/**
 * Registers /ws/topics — a single broadcast-only channel, every frame
 * shaped `{ type: <topic>, data }`. Clients subscribe to whichever topics
 * they care about client-side (RobotApp's src/api/topicsSocket.ts) rather
 * than negotiating subscriptions server-side, so adding a new topic here
 * is just one more `eventBus.on(...)` line — no protocol change. Replaces
 * the old WebServer's /ws/autonav, whose ROS callbacks called
 * `asyncio.run()` on every single tick to push onto it; here the ROS
 * gateway only touches core/eventBus, and this is the one place that
 * turns those events into socket frames.
 */
export function registerTopicsWebSocket(wsRouter: WsRouter): void {
  const sockets = new Set<WebSocket>();

  const send = (payload: unknown) => {
    const message = JSON.stringify(payload);
    for (const socket of sockets) {
      if (socket.readyState === WebSocket.OPEN) {
        socket.send(message);
      }
    }
  };

  eventBus.on("map", (data) => send({ type: "map", data }));
  eventBus.on("location", (data) => send({ type: "location", data }));
  eventBus.on("navStatus", (data) => send({ type: "nav_status", data }));
  eventBus.on("robotStatus", (data) => send({ type: "robotStatus", data }));

  wsRouter.register("/ws/topics", (socket) => {
    sockets.add(socket);
    logger.debug({ clients: sockets.size }, "topics client connected");

    socket.on("close", () => {
      sockets.delete(socket);
      logger.debug({ clients: sockets.size }, "topics client disconnected");
    });
  });
}
