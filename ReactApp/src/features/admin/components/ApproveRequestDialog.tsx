import { useState } from "react";
import { Button } from "../../../components/ui/Button";
import { getApiErrorMessage } from "../../../app/apiError";
import { useApproveRequestMutation, useListUnassignedRobotsQuery } from "../adminApi";

interface ApproveRequestDialogProps {
  requestId: string;
  onClose: () => void;
}

export function ApproveRequestDialog({ requestId, onClose }: ApproveRequestDialogProps) {
  const { data: robots, isLoading: isLoadingRobots } = useListUnassignedRobotsQuery();
  const [robotId, setRobotId] = useState("");
  const [approve, { isLoading, error }] = useApproveRequestMutation();

  async function handleApprove() {
    if (!robotId) return;
    await approve({ id: requestId, robotId }).unwrap();
    onClose();
  }

  return (
    <div className="fixed inset-0 flex items-center justify-center bg-black/40 px-4">
      <div className="w-full max-w-sm rounded-xl bg-white p-6 shadow-lg dark:bg-slate-800">
        <h2 className="mb-4 text-lg font-semibold text-slate-900 dark:text-slate-100">
          Assign a robot
        </h2>

        {isLoadingRobots ? (
          <p className="text-sm text-slate-500 dark:text-slate-400">Loading unassigned robots…</p>
        ) : robots && robots.length > 0 ? (
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
            No unassigned robots available. Provision one first.
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
