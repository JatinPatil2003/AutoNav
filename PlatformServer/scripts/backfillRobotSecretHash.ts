import { connectDatabase, disconnectDatabase } from "../src/db/connection";
import { RobotModel } from "../src/modules/robots/robot.model";
import { decryptSecret, hashSecretForLookup } from "../src/common/utils/secretCrypto.util";
import { logger } from "../src/common/utils/logger.util";

/**
 * One-time migration: backfills `robotSecretHash` on robots provisioned
 * before requireRobotAuth switched from (robotId, robotSecret) to
 * (robotSecret) alone. Decrypts each robot's already-stored encrypted
 * secret (recoverable by design, see secretCrypto.util.ts) and stores its
 * SHA-256 fingerprint alongside it. Safe to re-run — skips robots that
 * already have a hash.
 *
 * Usage: npm run migrate:robot-secret-hash
 */
async function main(): Promise<void> {
  await connectDatabase();

  const robots = await RobotModel.find({
    $or: [{ robotSecretHash: { $exists: false } }, { robotSecretHash: null }],
  }).exec();

  logger.info(`backfillRobotSecretHash: ${robots.length} robot(s) need a secret hash`);

  for (const robot of robots) {
    const plaintext = decryptSecret(robot.robotSecretEncrypted);
    robot.robotSecretHash = hashSecretForLookup(plaintext);
    await robot.save();
    logger.info({ robotId: robot.robotId }, "backfillRobotSecretHash: backfilled");
  }

  logger.info("backfillRobotSecretHash: done");
}

main()
  .catch((err) => {
    logger.error({ err }, "backfillRobotSecretHash: failed");
    process.exitCode = 1;
  })
  .finally(async () => {
    await disconnectDatabase();
  });
