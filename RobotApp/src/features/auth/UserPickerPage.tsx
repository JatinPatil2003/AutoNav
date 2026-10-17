import { useEffect, useState } from "react";
import { ChevronRight, UserPlus, Wifi } from "lucide-react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { AppShell } from "../../components/ui/AppShell";
import { FullPageSpinner } from "../../components/ui/Spinner";
import { UserAvatar } from "../../components/ui/UserAvatar";
import { getApiErrorMessage } from "../../api/apiError";
import { hmiClient, RosterUser } from "../../api/hmiClient";
import { maskEmail } from "../../utils/maskEmail";

interface UserPickerPageProps {
  onPickUser: (email: string, name: string, pinSet: boolean, avatarUrl?: string) => void;
  onNewUser: () => void;
  onOpenWifi: () => void;
}

/**
 * The robot's lock screen: every reopen lands here first, showing who has
 * access right now so they just tap their name instead of typing an email.
 * "+ New user" still falls back to the email-based identify flow for
 * someone not yet in the cached roster.
 */
export function UserPickerPage({ onPickUser, onNewUser, onOpenWifi }: UserPickerPageProps) {
  const [users, setUsers] = useState<RosterUser[] | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    hmiClient
      .listUsers()
      .then((result) => setUsers(result.users))
      .catch((err) => setError(getApiErrorMessage(err)));
  }, []);

  if (!users && !error) return <FullPageSpinner />;

  return (
    <AppShell maxWidthClassName="max-w-md">
      <Card>
        <h1 className="mb-1 text-2xl font-bold tracking-tight text-slate-900 dark:text-slate-50">
          Who's this?
        </h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
          Select your account to log in.
        </p>

        {error && (
          <p className="mb-4 rounded-lg bg-red-50 px-3 py-2 text-sm text-red-600 dark:bg-red-900/20 dark:text-red-400">
            {error}
          </p>
        )}

        {users && users.length > 0 && (
          <ul className="mb-4 flex flex-col gap-2">
            {users.map((user) => (
              <li key={user.email}>
                <button
                  type="button"
                  onClick={() => onPickUser(user.email, user.name, user.pinSet, user.avatarUrl)}
                  className="group flex w-full items-center gap-3 rounded-xl border border-slate-200/70 bg-white/60 px-4 py-3 text-left transition-all hover:-translate-y-0.5 hover:border-brand-300 hover:bg-white hover:shadow-md hover:shadow-brand-900/5 dark:border-slate-700/70 dark:bg-slate-800/40 dark:hover:bg-slate-800"
                >
                  <UserAvatar name={user.name} avatarUrl={user.avatarUrl} />
                  <span className="min-w-0 flex-1">
                    <span className="block truncate text-sm font-semibold text-slate-900 dark:text-slate-100">
                      {user.name}
                    </span>
                    <span className="block truncate text-xs text-slate-500 dark:text-slate-400">
                      {maskEmail(user.email)}
                    </span>
                  </span>
                  <ChevronRight className="h-4 w-4 flex-shrink-0 text-slate-300 transition-all group-hover:translate-x-0.5 group-hover:text-brand-500" />
                </button>
              </li>
            ))}
          </ul>
        )}

        {users && users.length === 0 && (
          <p className="mb-4 text-sm text-slate-500 dark:text-slate-400">
            No users have access to this robot yet.
          </p>
        )}

        <Button variant="secondary" className="w-full" onClick={onNewUser}>
          <UserPlus className="h-4 w-4" /> + New user
        </Button>
        <Button variant="ghost" className="mt-2 w-full" onClick={onOpenWifi}>
          <Wifi className="h-4 w-4" /> Wifi settings
        </Button>
      </Card>
    </AppShell>
  );
}
