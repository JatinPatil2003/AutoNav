import { createApp } from "./app";
import { config } from "./config/env";
import { connectDatabase } from "./db/connection";
import { logger } from "./common/utils/logger.util";

async function bootstrap(): Promise<void> {
  await connectDatabase();

  const app = createApp();

  app.listen(config.port, () => {
    logger.info(`PlatformServer listening on port ${config.port} (${config.nodeEnv})`);
  });
}

bootstrap().catch((err) => {
  logger.error({ err }, "Failed to start PlatformServer");
  process.exit(1);
});
