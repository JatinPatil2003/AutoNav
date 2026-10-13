import { connectDatabase, disconnectDatabase } from "../src/db/connection";
import { RobotModel } from "../src/modules/robots/robot.model";
import { logger } from "../src/common/utils/logger.util";

/**
 * One-off dev/ops helper to provision a Robot record so an admin has
 * something to approve robot-access requests against. Usage:
 *   npm run seed:robot -- --name=autonav --hostname=autonav.robot
 */
async function main(): Promise<void> {
  const args = Object.fromEntries(
    process.argv.slice(2).map((arg) => {
      const [key, value] = arg.replace(/^--/, "").split("=");
      return [key, value];
    }),
  );

  const name = args.name ?? "autonav";
  const vpnHostname = args.hostname ?? `${name}.robot`;

  await connectDatabase();

  const existing = await RobotModel.findOne({ name });
  if (existing) {
    logger.info(`Robot "${name}" already exists (id=${existing._id.toString()}), skipping.`);
  } else {
    const robot = await RobotModel.create({ name, vpnHostname, status: "unassigned" });
    logger.info(`Created robot "${name}" (id=${robot._id.toString()}, vpnHostname=${vpnHostname})`);
  }

  await disconnectDatabase();
}

main().catch((err) => {
  logger.error({ err }, "seedRobot failed");
  process.exit(1);
});
