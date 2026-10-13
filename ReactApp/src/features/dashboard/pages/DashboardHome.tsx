import { Card } from "../../../components/ui/Card";
import { useAuth } from "../../auth/useAuth";
import { SetPasswordForm } from "../../auth/components/SetPasswordForm";

export function DashboardHome() {
  const { user } = useAuth();
  const hasLocalPassword = user?.authProviders.includes("local") ?? true;

  return (
    <div className="p-6">
      <h1 className="mb-4 text-xl font-semibold text-slate-900 dark:text-slate-100">Welcome, {user?.name}</h1>
      <Card>
        <h2 className="mb-2 text-sm font-medium text-slate-500 dark:text-slate-400">Assigned robot</h2>
        <p className="text-lg font-semibold text-slate-900 dark:text-slate-100">
          {user?.robot?.displayName ?? user?.robot?.name}
        </p>
        <p className="text-sm text-slate-500 dark:text-slate-400">{user?.robot?.vpnHostname}</p>
        <p className="mt-1 text-sm capitalize text-slate-500 dark:text-slate-400">Status: {user?.robot?.status}</p>
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
