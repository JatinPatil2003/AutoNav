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
  // /ws/robot-link is kept unreachable from the public internet at the
  // nginx layer (see AWS/nginx/platform-server.autonav.in.conf) — it is
  // only ever dialed over the VPN, directly at platform.internal:4000.
  wsRouter.attach(server);

  server.listen(config.port, () => {
    logger.info(`PlatformServer listening on port ${config.port} (${config.nodeEnv})`);
  });
}

bootstrap().catch((err) => {
  logger.error({ err }, "Failed to start PlatformServer");
  process.exit(1);
});
