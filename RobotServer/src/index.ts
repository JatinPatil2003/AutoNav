import http from "node:http";
import { createApp } from "./app";
import { config } from "./config/env";
import { connectDatabase } from "./db/connection";
import { logger } from "./common/utils/logger.util";
import { initRosNode, shutdownRosNode } from "./ros/rosNode";
import { RosServiceGateway } from "./ros/rosServiceGateway";
import { DeviceIdentityService } from "./ros/deviceIdentity";
import { wsRouter } from "./core/wsRouter";
import { verifyAccessTokenFromCookieHeader } from "./common/middleware/auth.middleware";
import { PlatformLinkClient } from "./clients/platformLink.client";

const DEVICE_ID_BOOT_ATTEMPTS = 3;

async function bootstrap(): Promise<void> {
  await connectDatabase();
  const rosNode = await initRosNode();

  // Who this robot is, resolved from the ESP32 bridge — not config. If the
  // bridge isn't up yet, boot continues in a degraded state (ROS/local PIN
  // login must still come up even if the ESP32 or VPN is down) and
  // resolution is retried lazily on every periodic sync tick below.
  const deviceIdentity = new DeviceIdentityService(new RosServiceGateway(rosNode));
  const robotId = await deviceIdentity.resolveWithRetries(DEVICE_ID_BOOT_ATTEMPTS);
  if (!robotId) {
    logger.warn("Device id not resolved at boot — continuing degraded until the ESP32 bridge responds");
  }

  const { app, registerWebSockets, registerPlatformLink, teleopService, rosterSyncService } = createApp(
    rosNode,
    deviceIdentity,
  );
  const server = http.createServer(app);

  registerWebSockets(wsRouter);
  wsRouter.attach(server, (request) => !!verifyAccessTokenFromCookieHeader(request.headers.cookie));

  // Persistent outbound link to PlatformServer's /ws/robot-link — this is
  // what makes "robot online" a push-based fact there. Wired to
  // teleopLink.handler via registerPlatformLink so a lost/never-made link
  // fails safe through teleopService.stop() (see platformLink.client.ts).
  const platformLink = new PlatformLinkClient(deviceIdentity, teleopService);
  registerPlatformLink(platformLink);
  platformLink.start();

  // Best-effort — if PlatformServer/the VPN is unreachable at boot, ROS
  // functionality must still come up; PIN login just keeps serving
  // whatever was cached from the last successful sync (see
  // localUser.model.ts's persisted, not in-memory-only, cache).
  rosterSyncService.sync().catch((err) => {
    logger.warn({ err }, "Initial roster sync failed — continuing with the last cached roster, if any");
  });
  setInterval(
    () => {
      rosterSyncService.sync().catch((err) => logger.warn({ err }, "Periodic roster sync failed"));
    },
    config.rosterSyncIntervalMinutes * 60 * 1000,
  ).unref();

  server.listen(config.port, () => {
    logger.info(`RobotServer listening on port ${config.port} (${config.nodeEnv})`);
  });

  const shutdown = async (): Promise<void> => {
    logger.info("Shutting down RobotServer");
    platformLink.stop();
    server.close();
    await shutdownRosNode();
    process.exit(0);
  };

  process.on("SIGTERM", shutdown);
  process.on("SIGINT", shutdown);
}

bootstrap().catch((err) => {
  logger.error({ err }, "Failed to start RobotServer");
  process.exit(1);
});
