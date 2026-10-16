import { Bot, Wifi, WifiOff } from "lucide-react";
import { Card } from "../../../components/ui/Card";
import { useAuth } from "../../auth/useAuth";
import { RobotSummary } from "../../auth/auth.types";
import { SetPasswordForm } from "../../auth/components/SetPasswordForm";

export function DashboardHome() {
  const { user } = useAuth();
  const hasLocalPassword = user?.authProviders.includes("local") ?? true;
  const robots = user?.robots ?? [];

  return (
    <div className="p-3">
      <h1 className="mb-4 text-xl font-semibold text-slate-900 dark:text-slate-100">
        Welcome, {user?.name}
      </h1>
      <Card>
        <h2 className="mb-3 text-sm font-medium text-slate-500 dark:text-slate-400">
          Assigned robot{robots.length !== 1 ? "s" : ""}
        </h2>
        {robots.length === 0 && (
          <p className="text-sm text-slate-500 dark:text-slate-400">No robot assigned yet.</p>
        )}
        <div className="flex flex-col gap-2">
          {robots.map((robot: RobotSummary) => {
            const online = robot.status === "online";
            return (
              <div
                key={robot.id}
                className="flex items-center gap-3 rounded-xl border border-slate-200/70 bg-white/60 px-4 py-3 dark:border-slate-700/70 dark:bg-slate-800/40"
              >
                <span className="flex h-10 w-10 flex-shrink-0 items-center justify-center rounded-lg bg-gradient-to-br from-brand-400 to-brand-600 shadow-sm shadow-brand-900/20">
                  <Bot className="h-5 w-5 text-white" />
                </span>
                <div className="min-w-0 flex-1">
                  <p className="truncate text-sm font-semibold text-slate-900 dark:text-slate-100">
                    {robot.displayName ?? robot.name}
                  </p>
                  <p className="truncate text-xs text-slate-500 dark:text-slate-400">{robot.vpnHostname}</p>
                </div>
                <span
                  className={`flex items-center gap-1 rounded-full px-2 py-1 text-xs font-medium capitalize ${
                    online
                      ? "bg-emerald-50 text-emerald-600 dark:bg-emerald-500/10 dark:text-emerald-400"
                      : "bg-slate-100 text-slate-500 dark:bg-slate-700/50 dark:text-slate-400"
                  }`}
                >
                  {online ? <Wifi className="h-3.5 w-3.5" /> : <WifiOff className="h-3.5 w-3.5" />}
                  {robot.status}
                </span>
              </div>
            );
          })}
        </div>
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
