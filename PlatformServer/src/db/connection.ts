import mongoose from "mongoose";
import { config } from "../config/env";
import { logger } from "../common/utils/logger.util";

mongoose.connection.on("connected", () => {
  logger.info("MongoDB connection established");
});

mongoose.connection.on("error", (err) => {
  logger.error({ err }, "MongoDB connection error");
});

mongoose.connection.on("disconnected", () => {
  logger.warn("MongoDB connection lost");
});

export async function connectDatabase(): Promise<typeof mongoose> {
  return mongoose.connect(config.mongoUri);
}

export async function disconnectDatabase(): Promise<void> {
  await mongoose.disconnect();
}
