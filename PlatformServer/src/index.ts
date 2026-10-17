import http from "node:http";
import { createApp } from "./app";
import { config } from "./config/env";
import { connectDatabase } from "./db/connection";
import { logger } from "./common/utils/logger.util";

async function bootstrap(): Promise<void> {
  await connectDatabase();

  const { app, wsRouter } = createApp();
  const server = http.createServer(app);

  // Both /ws/robot-link and /ws/control attach to this one listener.
  // /ws/robot-link is now proxied publicly over HTTPS at
  // https://platform-server.autonav.in/ws/robot-link (see
  // AWS/nginx/platform-server.autonav.in.conf) — auth is enforced by
  // ROBOT_LINK_SECRET (robotLinkGateway.authenticate), not a VPN boundary.
  wsRouter.attach(server);

  server.listen(config.port, () => {
    logger.info(`PlatformServer listening on port ${config.port} (${config.nodeEnv})`);
  });
}

bootstrap().catch((err) => {
  logger.error({ err }, "Failed to start PlatformServer");
  process.exit(1);
});
