import pino from "pino";
import { config } from "../../config/env";

const level = config.nodeEnv === "test" ? "silent" : config.isProduction ? "info" : "debug";

export const logger = pino({
  level,
  transport:
    config.nodeEnv === "test"
      ? undefined
      : config.isProduction
        ? undefined
        : { target: "pino-pretty", options: { colorize: true } },
});
