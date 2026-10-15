import { Card } from "../../../components/ui/Card";
import { useAuth } from "../../auth/useAuth";
import { RobotSummary } from "../../auth/auth.types";
import { SetPasswordForm } from "../../auth/components/SetPasswordForm";

export function DashboardHome() {
  const { user } = useAuth();
  const hasLocalPassword = user?.authProviders.includes("local") ?? true;
  const robots = user?.robots ?? [];

  return (
    <div className="p-6">
      <h1 className="mb-4 text-xl font-semibold text-slate-900 dark:text-slate-100">Welcome, {user?.name}</h1>
      <Card>
        <h2 className="mb-2 text-sm font-medium text-slate-500 dark:text-slate-400">
          Assigned robot{robots.length !== 1 ? "s" : ""}
        </h2>
        {robots.length === 0 && (
          <p className="text-sm text-slate-500 dark:text-slate-400">No robot assigned yet.</p>
        )}
        {robots.map((robot: RobotSummary) => (
          <div key={robot.id} className="border-b border-slate-100 py-2 last:border-0 dark:border-slate-700">
            <p className="text-lg font-semibold text-slate-900 dark:text-slate-100">
              {robot.displayName ?? robot.name}
            </p>
            <p className="text-sm text-slate-500 dark:text-slate-400">{robot.vpnHostname}</p>
            <p className="mt-1 text-sm capitalize text-slate-500 dark:text-slate-400">Status: {robot.status}</p>
          </div>
        ))}
      </Card>

      {!hasLocalPassword && (
        <Card className="mt-6 max-w-sm">
          <h2 className="mb-2 text-sm font-medium text-slate-500 dark:text-slate-400">Add a password</h2>
          <p className="mb-4 text-sm text-slate-500 dark:text-slate-400">
            Your account currently only signs in via Google. Set a password to also sign in with
            email/password.
          </p>
          <SetPasswordForm />
        </Card>
      )}
    </div>
  );
}
