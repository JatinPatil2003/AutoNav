import { connectDatabase, disconnectDatabase } from "../src/db/connection";
import { RobotModel } from "../src/modules/robots/robot.model";
import { encryptSecret, generateRobotSecret, hashSecretForLookup } from "../src/common/utils/secretCrypto.util";
import { logger } from "../src/common/utils/logger.util";

/**
 * One-off dev/ops helper to provision a Robot record so an admin has
 * something to approve robot-access requests against. Usage:
 *   npm run seed:robot -- --robotId=autonav-01 --name=autonav --hostname=autonav.robot
 *
 * Prints the plaintext robotSecret once — copy it into that robot's own
 * RobotServer .env (PLATFORM_ROBOT_SECRET) now; it cannot be retrieved again.
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
    const robotSecret = generateRobotSecret();
    const robot = await RobotModel.create({
      robotId: robotId.toLowerCase(),
      name,
      vpnHostname,
      status: "provisioned",
      robotSecretEncrypted: encryptSecret(robotSecret),
      robotSecretHash: hashSecretForLookup(robotSecret),
    });
    logger.info(
      `Created robot "${name}" (id=${robot._id.toString()}, robotId=${robot.robotId}, vpnHostname=${vpnHostname})`,
    );
    // eslint-disable-next-line no-console -- intentionally printed once, not logged/retained
    console.log(`\nrobotSecret (copy into the robot's RobotServer .env, shown only once):\n${robotSecret}\n`);
  }

  await disconnectDatabase();
}

main().catch((err) => {
  logger.error({ err }, "seedRobot failed");
  process.exit(1);
});
