import { useState } from "react";
import { Button } from "../../../components/ui/Button";
import { getApiErrorMessage } from "../../../app/apiError";
import { RobotAccessRequest } from "../admin.types";
import { useApproveRequestMutation, useListAllRobotsQuery } from "../adminApi";

interface ApproveRequestDialogProps {
  request: RobotAccessRequest;
  onClose: () => void;
}

/** Defaults to the originally requested robot; the admin can still redirect it to a different one. */
export function ApproveRequestDialog({ request, onClose }: ApproveRequestDialogProps) {
  const { data: allRobots, isLoading: isLoadingRobots } = useListAllRobotsQuery();
  const robots = allRobots?.filter((robot) => robot.status !== "retired") ?? [];
  const [robotId, setRobotId] = useState(request.robot?.id ?? "");
  const [approve, { isLoading, error }] = useApproveRequestMutation();

  async function handleApprove() {
    if (!robotId) return;
    await approve({ id: request.id, robotId }).unwrap();
    onClose();
  }

  return (
    <div className="fixed inset-0 flex items-center justify-center bg-black/40 px-4">
      <div className="w-full max-w-sm rounded-xl bg-white p-6 shadow-lg dark:bg-slate-800">
        <h2 className="mb-4 text-lg font-semibold text-slate-900 dark:text-slate-100">
          Assign a robot
        </h2>

        {isLoadingRobots ? (
          <p className="text-sm text-slate-500 dark:text-slate-400">Loading robots…</p>
        ) : robots.length > 0 ? (
          <select
            value={robotId}
            onChange={(e) => setRobotId(e.target.value)}
            className="w-full rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 dark:border-slate-600 dark:bg-slate-900 dark:text-slate-100"
          >
            <option value="">Select a robot…</option>
            {robots.map((robot) => (
              <option key={robot.id} value={robot.id}>
                {robot.displayName ?? robot.name} ({robot.vpnHostname})
              </option>
            ))}
          </select>
        ) : (
          <p className="text-sm text-slate-500 dark:text-slate-400">
            No robots provisioned yet. Provision one first.
          </p>
        )}

        {error && (
          <p className="mt-3 text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(error)}</p>
        )}

        <div className="mt-6 flex justify-end gap-2">
          <Button variant="ghost" onClick={onClose}>
            Cancel
          </Button>
          <Button isLoading={isLoading} disabled={!robotId} onClick={handleApprove}>
            Approve
          </Button>
        </div>
      </div>
    </div>
  );
}
