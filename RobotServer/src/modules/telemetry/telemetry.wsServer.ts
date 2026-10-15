import { WebSocket } from "ws";
import { eventBus } from "../../core/eventBus";
import { WsRouter } from "../../core/wsRouter";
import { logger } from "../../common/utils/logger.util";

/**
 * Registers /ws/telemetry — a broadcast-only channel pushing map/location/
 * nav-status updates to every connected dashboard. Replaces the old
 * WebServer's /ws/autonav, whose ROS callbacks called `asyncio.run()` on
 * every single tick to push onto it; here the ROS gateway only touches
 * core/eventBus, and this is the one place that turns those events into
 * socket frames.
 */
export function registerTelemetryWebSocket(wsRouter: WsRouter): void {
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

  wsRouter.register("/ws/telemetry", (socket) => {
    sockets.add(socket);
    logger.debug({ clients: sockets.size }, "telemetry client connected");

    socket.on("close", () => {
      sockets.delete(socket);
      logger.debug({ clients: sockets.size }, "telemetry client disconnected");
    });
  });
}
