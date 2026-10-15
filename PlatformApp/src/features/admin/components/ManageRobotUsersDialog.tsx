import { useState } from "react";
import { Button } from "../../../components/ui/Button";
import { getApiErrorMessage } from "../../../app/apiError";
import { AdminRobot } from "../admin.types";
import {
  useGrantRobotAssignmentMutation,
  useListAssignmentsForRobotQuery,
  useListUsersQuery,
  useRevokeRobotAssignmentMutation,
  useUpdateRobotAssignmentAccessMutation,
} from "../adminApi";

interface ManageRobotUsersDialogProps {
  robot: AdminRobot;
  onClose: () => void;
}

/** Assign/unassign users for a single robot — the robot-centric side of the many-to-many relation. */
export function ManageRobotUsersDialog({ robot, onClose }: ManageRobotUsersDialogProps) {
  const { data: assignments, isLoading: isLoadingAssignments } = useListAssignmentsForRobotQuery(robot.id);
  const { data: allUsers, isLoading: isLoadingUsers } = useListUsersQuery();
  const [grant, { isLoading: isGranting, error: grantError }] = useGrantRobotAssignmentMutation();
  const [revoke, { isLoading: isRevoking, error: revokeError }] = useRevokeRobotAssignmentMutation();
  const [updateAccess, { error: accessError }] = useUpdateRobotAssignmentAccessMutation();
  const [userId, setUserId] = useState("");
  const [hmiAccess, setHmiAccess] = useState(true);
  const [platformAccess, setPlatformAccess] = useState(true);

  const activeAssignments = (assignments ?? []).filter((a) => a.status === "active");
  const assignedUserIds = new Set(activeAssignments.map((a) => a.user?.id));
  const availableUsers = (allUsers ?? []).filter((user) => !assignedUserIds.has(user.id));

  async function handleGrant() {
    if (!userId) return;
    await grant({ userId, robotId: robot.id, hmiAccess, platformAccess }).unwrap();
    setUserId("");
    setHmiAccess(true);
    setPlatformAccess(true);
  }

  async function handleRevoke(assignmentId: string) {
    await revoke({ id: assignmentId }).unwrap();
  }

  return (
    <div className="fixed inset-0 flex items-center justify-center bg-black/40 px-4">
      <div className="w-full max-w-md rounded-xl bg-white p-6 shadow-lg dark:bg-slate-800">
        <h2 className="mb-1 text-lg font-semibold text-slate-900 dark:text-slate-100">Manage users</h2>
        <p className="mb-4 text-sm text-slate-500 dark:text-slate-400">
          {robot.displayName ?? robot.name} ({robot.robotId})
        </p>

        {isLoadingAssignments ? (
          <p className="text-sm text-slate-500 dark:text-slate-400">Loading…</p>
        ) : activeAssignments.length === 0 ? (
          <p className="text-sm text-slate-500 dark:text-slate-400">No users assigned yet.</p>
        ) : (
          <ul className="mb-4 flex flex-col gap-2">
            {activeAssignments.map((assignment) => (
              <li
                key={assignment.id}
                className="rounded-md border border-slate-200 px-3 py-2 text-sm dark:border-slate-700"
              >
                <div className="mb-2 flex items-center justify-between">
                  <span className="text-slate-900 dark:text-slate-100">
                    {assignment.user?.name}{" "}
                    <span className="text-slate-400 dark:text-slate-500">({assignment.user?.email})</span>
                  </span>
                  <Button
                    variant="secondary"
                    isLoading={isRevoking}
                    onClick={() => handleRevoke(assignment.id)}
                  >
                    Unassign
                  </Button>
                </div>
                <div className="flex gap-4 text-xs text-slate-500 dark:text-slate-400">
                  <label className="flex items-center gap-1.5">
                    <input
                      type="checkbox"
                      checked={assignment.hmiAccess}
                      onChange={(e) =>
                        updateAccess({ id: assignment.id, hmiAccess: e.target.checked })
                      }
                    />
                    HMI (robot PIN login)
                  </label>
                  <label className="flex items-center gap-1.5">
                    <input
                      type="checkbox"
                      checked={assignment.platformAccess}
                      onChange={(e) =>
                        updateAccess({ id: assignment.id, platformAccess: e.target.checked })
                      }
                    />
                    PlatformApp
                  </label>
                </div>
              </li>
            ))}
          </ul>
        )}

        {revokeError && (
          <p className="mb-3 text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(revokeError)}</p>
        )}
        {accessError && (
          <p className="mb-3 text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(accessError)}</p>
        )}

        <div className="border-t border-slate-200 pt-4 dark:border-slate-700">
          <h3 className="mb-2 text-sm font-medium text-slate-500 dark:text-slate-400">Assign a user</h3>
          {isLoadingUsers ? (
            <p className="text-sm text-slate-500 dark:text-slate-400">Loading users…</p>
          ) : availableUsers.length > 0 ? (
            <div className="flex flex-col gap-2">
              <div className="flex gap-2">
                <select
                  value={userId}
                  onChange={(e) => setUserId(e.target.value)}
                  className="flex-1 rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 dark:border-slate-600 dark:bg-slate-900 dark:text-slate-100"
                >
                  <option value="">Select a user…</option>
                  {availableUsers.map((user) => (
                    <option key={user.id} value={user.id}>
                      {user.name} ({user.email})
                    </option>
                  ))}
                </select>
                <Button isLoading={isGranting} disabled={!userId} onClick={handleGrant}>
                  Assign
                </Button>
              </div>
              <div className="flex gap-4 text-xs text-slate-500 dark:text-slate-400">
                <label className="flex items-center gap-1.5">
                  <input
                    type="checkbox"
                    checked={hmiAccess}
                    onChange={(e) => setHmiAccess(e.target.checked)}
                  />
                  HMI (robot PIN login)
                </label>
                <label className="flex items-center gap-1.5">
                  <input
                    type="checkbox"
                    checked={platformAccess}
                    onChange={(e) => setPlatformAccess(e.target.checked)}
                  />
                  PlatformApp
                </label>
              </div>
            </div>
          ) : (
            <p className="text-sm text-slate-500 dark:text-slate-400">
              No other users available to assign.
            </p>
          )}
          {grantError && (
            <p className="mt-2 text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(grantError)}</p>
          )}
        </div>

        <div className="mt-6 flex justify-end">
          <Button variant="ghost" onClick={onClose}>
            Close
          </Button>
        </div>
      </div>
    </div>
  );
}
