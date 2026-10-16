import crypto from "node:crypto";
import { connectDatabase, disconnectDatabase } from "../src/db/connection";
import { RobotModel } from "../src/modules/robots/robot.model";
import { logger } from "../src/common/utils/logger.util";

const LINK_SECRET_BYTES = 32;

function hashLinkSecret(secret: string): string {
  return crypto.createHash("sha256").update(secret).digest("hex");
}

/**
 * One-off dev/ops helper to provision a Robot record (and its robot-link
 * secret) so an admin has something to approve robot-access requests
 * against, and so a RobotServer instance has a secret to authenticate
 * /ws/robot-link with. Usage:
 *   npm run seed:robot -- --robotId=autonav-01 --name=autonav --hostname=autonav.robot [--secret=<hex>] [--rotate]
 *
 * `robotId` must match the device id RobotServer resolves for itself (see
 * RobotServer's ros/deviceIdentity.ts / DEV_DEVICE_ID_OVERRIDE).
 *
 * The plaintext secret is only ever shown here, once — only its sha256 hash
 * is persisted. Pass --secret=<hex> to pin it to a known dev value instead
 * of a random one (handy for keeping a local .env in sync across re-seeds);
 * pass --rotate to reissue a secret for a robot that already exists.
 */
async function main(): Promise<void> {
  const args = Object.fromEntries(
    process.argv.slice(2).map((arg) => {
      const [key, value] = arg.replace(/^--/, "").split("=");
      return [key, value ?? "true"];
    }),
  );

  const name = args.name ?? "autonav";
  const robotId = (args.robotId ?? name).toLowerCase();
  const vpnHostname = args.hostname ?? `${name}.robot`;

  await connectDatabase();

  let robot = await RobotModel.findOne({ robotId });
  const isNew = !robot;

  if (!robot) {
    robot = await RobotModel.create({
      robotId,
      name,
      vpnHostname,
      status: "provisioned",
    });
    logger.info(`Created robot "${name}" (id=${robot._id.toString()}, robotId=${robot.robotId})`);
  } else {
    logger.info(`Robot "${robotId}" already exists (id=${robot._id.toString()})`);
  }

  if (isNew || args.rotate === "true") {
    const linkSecret = args.secret ?? crypto.randomBytes(LINK_SECRET_BYTES).toString("hex");
    await RobotModel.updateOne(
      { _id: robot._id },
      { $set: { linkSecretHash: hashLinkSecret(linkSecret) } },
    ).exec();

    // eslint-disable-next-line no-console
    console.log(
      `\nROBOT_LINK_SECRET=${linkSecret}\n` +
        `(put this in RobotServer's .env for robotId="${robotId}" — shown once, not recoverable)\n`,
    );
  } else {
    logger.info("Robot already has a link secret; pass --rotate to reissue one.");
  }

  await disconnectDatabase();
}

main().catch((err) => {
  logger.error({ err }, "seedRobot failed");
  process.exit(1);
});
