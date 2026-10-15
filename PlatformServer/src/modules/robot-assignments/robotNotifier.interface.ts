/**
 * Abstraction over the platform->robot "operations" call (direction 2 of the
 * service-to-service JWT design): best-effort notifies a robot's own
 * RobotServer that its cached roster is stale and should be re-synced now
 * (e.g. right after an admin revokes a user's access). Implemented as a
 * no-op until the robot-auth module (which owns the per-robot secret
 * signing) is wired up; swapping in a real HTTP-calling implementation
 * requires no change to any caller.
 */
export interface IRobotNotifier {
  notifyRosterInvalidate(robotId: string): Promise<void>;
}

export class NoopRobotNotifier implements IRobotNotifier {
  async notifyRosterInvalidate(_robotId: string): Promise<void> {
    // Intentionally does nothing — see class doc comment.
  }
}
