import { useState } from "react";
import { useNavigate } from "react-router-dom";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { FullPageSpinner } from "../../../components/ui/Spinner";
import { getApiErrorMessage } from "../../../app/apiError";
import { useLogoutMutation } from "../../auth/authApi";
import { RequestStatusBanner } from "../components/RequestStatusBanner";
import {
  useCreateRobotAccessRequestMutation,
  useListMyRobotAccessRequestsQuery,
  useListRobotOptionsQuery,
} from "../robotAccessApi";

export function RequestAccessPage() {
  const { data: requests, isLoading: isLoadingRequests } = useListMyRobotAccessRequestsQuery();
  const { data: robotOptions, isLoading: isLoadingRobots } = useListRobotOptionsQuery();
  const [createRequest, { isLoading: isSubmitting, error }] = useCreateRobotAccessRequestMutation();
  const [logout] = useLogoutMutation();
  const [selectedRobotId, setSelectedRobotId] = useState("");
  const navigate = useNavigate();

  if (isLoadingRequests || isLoadingRobots) return <FullPageSpinner />;

  const pendingRobotIds = new Set(
    (requests ?? []).filter((r) => r.status === "pending").map((r) => r.robot?.id),
  );
  const requestableRobots = (robotOptions ?? []).filter((robot) => !pendingRobotIds.has(robot.id));

  async function handleLogout() {
    await logout();
    navigate("/login", { replace: true });
  }

  async function handleSubmit() {
    if (!selectedRobotId) return;
    await createRequest({ robotId: selectedRobotId }).unwrap();
    setSelectedRobotId("");
  }

  return (
    <div className="app-canvas flex min-h-screen items-center justify-center px-4">
      <Card className="w-full max-w-md text-center">
        <h1 className="mb-2 text-lg font-semibold text-slate-900 dark:text-slate-100">
          No robot assigned yet
        </h1>
        <p className="mb-4 text-sm text-slate-500 dark:text-slate-400">
          Pick a robot and request access — an administrator will review it.
        </p>

        {(requests ?? []).map((request) => (
          <RequestStatusBanner key={request.id} request={request} />
        ))}

        {requestableRobots.length > 0 ? (
          <div className="mt-4 flex flex-col gap-3 text-left">
            <select
              value={selectedRobotId}
              onChange={(e) => setSelectedRobotId(e.target.value)}
              className="w-full rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 dark:border-slate-600 dark:bg-slate-900 dark:text-slate-100"
            >
              <option value="">Select a robot…</option>
              {requestableRobots.map((robot) => (
                <option key={robot.id} value={robot.id}>
                  {robot.displayName ?? robot.name} ({robot.robotId})
                </option>
              ))}
            </select>

            {error && (
              <p className="text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(error)}</p>
            )}

            <Button isLoading={isSubmitting} disabled={!selectedRobotId} onClick={handleSubmit}>
              Request robot access
            </Button>
          </div>
        ) : (
          <p className="mt-4 text-sm text-slate-500 dark:text-slate-400">
            No robots available to request right now.
          </p>
        )}

        <Button variant="ghost" className="mt-3 w-full" onClick={handleLogout}>
          Log out
        </Button>
      </Card>
    </div>
  );
}
