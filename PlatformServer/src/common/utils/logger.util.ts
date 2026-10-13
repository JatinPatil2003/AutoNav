import pino, { TransportTargetOptions } from "pino";
import { config } from "../../config/env";

const level = config.nodeEnv === "test" ? "silent" : config.isProduction ? "info" : "debug";

function buildTransport(): { targets: TransportTargetOptions[] } | TransportTargetOptions | undefined {
  if (config.nodeEnv === "test") {
    return undefined;
  }

  if (config.isProduction) {
    return {
      targets: [
        // stdout — picked up by `docker logs` / whatever ships container
        // output (e.g. a CloudWatch agent) on the EC2 host.
        { target: "pino/file", level, options: {} },
        // Rotating file on disk: daily or 20MB, whichever comes first,
        // keeping the last 14 rotated files. pino-roll doesn't gzip on its
        // own — see AWS/README.md for the logrotate config that compresses
        // these further at the OS level.
        {
          target: "pino-roll",
          level,
          options: { file: "logs/app.log", frequency: "daily", size: "20m", mkdir: true, limit: { count: 14 } },
        },
      ],
    };
  }

  // Dev: pretty console only — no file, so local runs stay simple to read.
  return { target: "pino-pretty", level, options: { colorize: true } };
}

export const logger = pino({
  level,
  transport: buildTransport(),
});
