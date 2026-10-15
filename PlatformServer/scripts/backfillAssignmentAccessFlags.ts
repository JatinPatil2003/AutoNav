import { connectDatabase, disconnectDatabase } from "../src/db/connection";
import { RobotAssignmentModel } from "../src/modules/robot-assignments/robotAssignment.model";
import { logger } from "../src/common/utils/logger.util";

/**
 * One-time migration: backfills `hmiAccess`/`platformAccess` (both default
 * true) on assignments created before those per-assignment access flags
 * existed. Safe to re-run — only touches docs missing either field.
 *
 * Usage: npm run migrate:assignment-access-flags
 */
async function main(): Promise<void> {
  await connectDatabase();

  const result = await RobotAssignmentModel.updateMany(
    {
      $or: [{ hmiAccess: { $exists: false } }, { platformAccess: { $exists: false } }],
    },
    { $set: { hmiAccess: true, platformAccess: true } },
  ).exec();

  logger.info(`backfillAssignmentAccessFlags: updated ${result.modifiedCount} assignment(s)`);
}

main()
  .catch((err) => {
    logger.error({ err }, "backfillAssignmentAccessFlags: failed");
    process.exitCode = 1;
  })
  .finally(async () => {
    await disconnectDatabase();
  });
