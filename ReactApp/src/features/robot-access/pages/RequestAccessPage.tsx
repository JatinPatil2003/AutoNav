import { useNavigate } from "react-router-dom";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { FullPageSpinner } from "../../../components/ui/Spinner";
import { ThemeToggle } from "../../../components/ui/ThemeToggle";
import { getApiErrorMessage } from "../../../app/apiError";
import { useLogoutMutation } from "../../auth/authApi";
import { RequestStatusBanner } from "../components/RequestStatusBanner";
import { useCreateRobotAccessRequestMutation, useGetMyRobotAccessRequestQuery } from "../robotAccessApi";

export function RequestAccessPage() {
  const { data: request, isLoading } = useGetMyRobotAccessRequestQuery();
  const [createRequest, { isLoading: isSubmitting, error }] = useCreateRobotAccessRequestMutation();
  const [logout] = useLogoutMutation();
  const navigate = useNavigate();

  if (isLoading) return <FullPageSpinner />;

  const canRequest = !request || request.status === "rejected";

  async function handleLogout() {
    await logout();
    navigate("/login", { replace: true });
  }

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
          Request access and an administrator will assign a robot to your account.
        </p>

        {request && <RequestStatusBanner request={request} />}

        {error && (
          <p className="mt-3 text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(error)}</p>
        )}

        {canRequest && (
          <Button className="mt-4 w-full" isLoading={isSubmitting} onClick={() => createRequest()}>
            Request robot access
          </Button>
        )}

        <Button variant="ghost" className="mt-3 w-full" onClick={handleLogout}>
          Log out
        </Button>
      </Card>
    </div>
  );
}
