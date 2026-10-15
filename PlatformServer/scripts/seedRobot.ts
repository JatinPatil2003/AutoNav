import { connectDatabase, disconnectDatabase } from "../src/db/connection";
import { RobotModel } from "../src/modules/robots/robot.model";
import { logger } from "../src/common/utils/logger.util";

/**
 * One-off dev/ops helper to provision a Robot record so an admin has
 * something to approve robot-access requests against. Usage:
 *   npm run seed:robot -- --robotId=autonav-01 --name=autonav --hostname=autonav.robot
 *
 * No secret is issued — the robot identifies itself to PlatformServer with
 * its own device id, resolved from the ESP32 bridge at boot (see
 * RobotServer's ros/deviceIdentity.ts). `robotId` here must match that
 * device's id.
 */
async function main(): Promise<void> {
  const args = Object.fromEntries(
    process.argv.slice(2).map((arg) => {
      const [key, value] = arg.replace(/^--/, "").split("=");
      return [key, value];
    }),
  );

  const name = args.name ?? "autonav";
  const robotId = args.robotId ?? name;
  const vpnHostname = args.hostname ?? `${name}.robot`;

  await connectDatabase();

  const existing = await RobotModel.findOne({ robotId: robotId.toLowerCase() });
  if (existing) {
    logger.info(`Robot "${robotId}" already exists (id=${existing._id.toString()}), skipping.`);
  } else {
    const robot = await RobotModel.create({
      robotId: robotId.toLowerCase(),
      name,
      vpnHostname,
      status: "provisioned",
    });
    logger.info(
      `Created robot "${name}" (id=${robot._id.toString()}, robotId=${robot.robotId}, vpnHostname=${vpnHostname})`,
    );
  }

  await disconnectDatabase();
}

main().catch((err) => {
  logger.error({ err }, "seedRobot failed");
  process.exit(1);
});
