import { useState } from "react";
import { useNavigate } from "react-router-dom";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { FullPageSpinner } from "../../../components/ui/Spinner";
import { ThemeToggle } from "../../../components/ui/ThemeToggle";
import { getApiErrorMessage } from "../../../app/apiError";
import { useLogoutMutation } from "../../auth/authApi";
import { RequestStatusBanner } from "../components/RequestStatusBanner";
import {
  useCreateRobotAccessRequestMutation,
  useListMyRobotAccessRequestsQuery,
  useLazyLookupRobotQuery,
} from "../robotAccessApi";
import { RobotOption } from "../robotAccess.types";

export function RequestAccessPage() {
  const { data: requests, isLoading: isLoadingRequests } = useListMyRobotAccessRequestsQuery();
  const [lookupRobot, { isFetching: isLookingUp }] = useLazyLookupRobotQuery();
  const [createRequest, { isLoading: isSubmitting, error: submitError }] =
    useCreateRobotAccessRequestMutation();
  const [logout] = useLogoutMutation();
  const [robotIdInput, setRobotIdInput] = useState("");
  const [foundRobot, setFoundRobot] = useState<RobotOption | null>(null);
  const [notFound, setNotFound] = useState(false);
  const navigate = useNavigate();

  if (isLoadingRequests) return <FullPageSpinner />;

  const pendingRobotIds = new Set(
    (requests ?? []).filter((r) => r.status === "pending").map((r) => r.robot?.id),
  );

  async function handleVerify() {
    const robotId = robotIdInput.trim();
    if (!robotId) return;
    setFoundRobot(null);
    setNotFound(false);
    const robot = await lookupRobot(robotId).unwrap();
    if (robot) {
      setFoundRobot(robot);
    } else {
      setNotFound(true);
    }
  }

  async function handleSubmit() {
    if (!foundRobot) return;
    await createRequest({ robotId: foundRobot.id }).unwrap();
    setRobotIdInput("");
    setFoundRobot(null);
    setNotFound(false);
  }

  async function handleLogout() {
    await logout();
    navigate("/login", { replace: true });
  }

  const alreadyPending = foundRobot ? pendingRobotIds.has(foundRobot.id) : false;

  return (
    <div className="relative flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <div className="absolute right-4 top-4">
        <ThemeToggle />
      </div>
      <Card className="w-full max-w-md text-center">
        <h1 className="mb-2 text-lg font-semibold text-slate-900 dark:text-slate-100">
          No robot assigned yet
        </h1>
        <p className="mb-4 text-sm text-slate-500 dark:text-slate-400">
          Enter the robot ID printed on your robot to request access to it.
        </p>

        {(requests ?? []).map((request) => (
          <RequestStatusBanner key={request.id} request={request} />
        ))}

        <div className="mt-4 flex flex-col gap-3 text-left">
          <div className="flex gap-2">
            <input
              value={robotIdInput}
              onChange={(e) => {
                setRobotIdInput(e.target.value);
                setFoundRobot(null);
                setNotFound(false);
              }}
              placeholder="Robot ID"
              className="w-full rounded-md border border-slate-300 bg-white px-3 py-2 text-sm text-slate-900 dark:border-slate-600 dark:bg-slate-900 dark:text-slate-100"
            />
            <Button isLoading={isLookingUp} disabled={!robotIdInput.trim()} onClick={handleVerify}>
              Verify
            </Button>
          </div>

          {notFound && (
            <p className="text-sm text-red-600 dark:text-red-400">Robot not available.</p>
          )}

          {foundRobot && (
            <div className="rounded-md border border-slate-200 p-3 text-sm dark:border-slate-700">
              <p className="font-medium text-slate-900 dark:text-slate-100">
                {foundRobot.displayName ?? foundRobot.name} ({foundRobot.robotId})
              </p>
              {alreadyPending ? (
                <p className="mt-1 text-slate-500 dark:text-slate-400">
                  You already have a pending request for this robot.
                </p>
              ) : (
                <>
                  {submitError && (
                    <p className="mt-1 text-red-600 dark:text-red-400">
                      {getApiErrorMessage(submitError)}
                    </p>
                  )}
                  <Button
                    isLoading={isSubmitting}
                    className="mt-2 w-full"
                    onClick={handleSubmit}
                  >
                    Request robot access
                  </Button>
                </>
              )}
            </div>
          )}
        </div>

        <Button variant="ghost" className="mt-3 w-full" onClick={handleLogout}>
          Log out
        </Button>
      </Card>
    </div>
  );
}
