import { useState } from "react";
import { Button } from "../../../components/ui/Button";
import { getApiErrorMessage } from "../../../app/apiError";
import { AdminRobot } from "../admin.types";
import { useDeleteRobotMutation, useListAllRobotsQuery } from "../adminApi";
import { ManageRobotUsersDialog } from "./ManageRobotUsersDialog";

const statusStyles: Record<string, string> = {
  provisioned: "bg-slate-100 text-slate-600 dark:bg-slate-700 dark:text-slate-300",
  active: "bg-green-100 text-green-700 dark:bg-green-900/40 dark:text-green-300",
  retired: "bg-red-100 text-red-700 dark:bg-red-900/40 dark:text-red-300",
};

export function RobotsList() {
  const { data: robots, isLoading } = useListAllRobotsQuery();
  const [managingRobot, setManagingRobot] = useState<AdminRobot | null>(null);
  const [deleteRobot, { isLoading: isDeleting, error: deleteError }] = useDeleteRobotMutation();

  async function handleDelete(robot: AdminRobot) {
    const confirmed = window.confirm(
      `Permanently delete "${robot.displayName ?? robot.name}" (${robot.robotId})? This also removes all of its user assignments and access requests.`,
    );
    if (!confirmed) return;
    await deleteRobot(robot.id).unwrap();
  }

  if (isLoading) return <p className="text-sm text-slate-500 dark:text-slate-400">Loading…</p>;

  if (!robots || robots.length === 0) {
    return (
      <p className="text-sm text-slate-500 dark:text-slate-400">No robots provisioned yet.</p>
    );
  }

  return (
    <>
      <ul className="flex flex-col gap-2">
        {robots.map((robot) => (
          <li
            key={robot.id}
            className="flex items-center justify-between rounded-md border border-slate-200 px-3 py-2 text-sm dark:border-slate-700"
          >
            <span>
              <span className="font-medium text-slate-900 dark:text-slate-100">
                {robot.displayName ?? robot.name}
              </span>{" "}
              <span className="text-slate-400 dark:text-slate-500">
                ({robot.robotId}, {robot.vpnHostname})
              </span>
            </span>
            <div className="flex items-center gap-2">
              <span
                className={`rounded-full px-2 py-0.5 text-xs font-medium capitalize ${
                  statusStyles[robot.status] ?? "bg-slate-100 text-slate-600 dark:bg-slate-700 dark:text-slate-300"
                }`}
              >
                {robot.status}
              </span>
              <Button variant="secondary" onClick={() => setManagingRobot(robot)}>
                Manage users
              </Button>
              <Button variant="danger" isLoading={isDeleting} onClick={() => handleDelete(robot)}>
                Delete
              </Button>
            </div>
          </li>
        ))}
      </ul>
      {deleteError && (
        <p className="mt-2 text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(deleteError)}</p>
      )}
      {managingRobot && (
        <ManageRobotUsersDialog robot={managingRobot} onClose={() => setManagingRobot(null)} />
      )}
    </>
  );
}
