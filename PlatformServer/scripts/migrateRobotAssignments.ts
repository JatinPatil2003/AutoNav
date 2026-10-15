import { Types } from "mongoose";
import { connectDatabase, disconnectDatabase } from "../src/db/connection";
import { RobotAssignmentModel } from "../src/modules/robot-assignments/robotAssignment.model";
import { RobotAccessRequestModel } from "../src/modules/robot-access-requests/robotAccessRequest.model";
import { logger } from "../src/common/utils/logger.util";

/**
 * One-time migration from the old one-to-one User.robot/Robot.assignedTo
 * fields to the new many-to-many RobotAssignment junction collection.
 *
 * Both old fields are read via the raw driver (not the Mongoose models,
 * which no longer declare them) since they've already been dropped from the
 * TS schemas in this same change. Safe to re-run: upserts on {user, robot},
 * so running it twice does not create duplicates.
 *
 * Also closes out any pre-existing pending RobotAccessRequest docs that
 * predate the `robot` field (nothing sensible to migrate them to — the
 * request never recorded which robot was wanted) as "rejected", rather than
 * guessing a target robot.
 *
 * Usage: npm run migrate:robot-assignments
 */
async function main(): Promise<void> {
  await connectDatabase();
  const db = RobotAssignmentModel.db.db;
  if (!db) throw new Error("No database connection available");

  const users = await db
    .collection("users")
    .find({ robot: { $exists: true, $ne: null } })
    .toArray();

  let migrated = 0;
  for (const user of users) {
    const userId = (user._id as Types.ObjectId).toString();
    const robotId = (user.robot as Types.ObjectId).toString();

    await RobotAssignmentModel.updateOne(
      { user: userId, robot: robotId },
      {
        $setOnInsert: {
          user: userId,
          robot: robotId,
          status: "active",
          pinUpdatedCount: 0,
          grantedBy: userId, // no admin attribution recorded historically; self-attributed as a marker
          grantedAt: user.updatedAt ?? new Date(),
        },
      },
      { upsert: true },
    );
    migrated += 1;
  }
  logger.info(`migrateRobotAssignments: migrated ${migrated} legacy user.robot assignment(s)`);

  const legacyPendingResult = await RobotAccessRequestModel.updateMany(
    { status: "pending", robot: { $exists: false } },
    {
      $set: {
        status: "rejected",
        rejectionReason: "Superseded by the multi-robot migration (no robot was recorded on this request)",
        reviewedAt: new Date(),
      },
    },
  );
  logger.info(
    `migrateRobotAssignments: closed out ${legacyPendingResult.modifiedCount} legacy robot-less pending request(s)`,
  );

  await disconnectDatabase();
}

main().catch((err) => {
  logger.error({ err }, "migrateRobotAssignments failed");
  process.exit(1);
});
