import { useEffect, useState } from "react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { FullPageSpinner } from "../../components/ui/Spinner";
import { UserAvatar } from "../../components/ui/UserAvatar";
import { getApiErrorMessage } from "../../api/apiError";
import { hmiClient, RosterUser } from "../../api/hmiClient";
import { maskEmail } from "../../utils/maskEmail";

interface UserPickerPageProps {
  onPickUser: (email: string, name: string, avatarUrl?: string) => void;
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
    <div className="flex min-h-screen flex-col items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <Card className="w-full max-w-md">
        <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">Who's this?</h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">Select your account to log in.</p>

        {error && <p className="mb-4 text-sm text-red-600 dark:text-red-400">{error}</p>}

        {users && users.length > 0 && (
          <ul className="mb-4 flex flex-col gap-2">
            {users.map((user) => (
              <li key={user.email}>
                <button
                  type="button"
                  onClick={() => onPickUser(user.email, user.name, user.avatarUrl)}
                  className="flex w-full items-center gap-3 rounded-md border border-slate-200 px-4 py-3 text-left transition-colors hover:bg-slate-100 dark:border-slate-700 dark:hover:bg-slate-800"
                >
                  <UserAvatar name={user.name} avatarUrl={user.avatarUrl} />
                  <span>
                    <span className="block text-sm font-medium text-slate-900 dark:text-slate-100">
                      {user.name}
                    </span>
                    <span className="block text-xs text-slate-500 dark:text-slate-400">
                      {maskEmail(user.email)}
                    </span>
                  </span>
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
          + New user
        </Button>
        <Button variant="ghost" className="mt-2 w-full" onClick={onOpenWifi}>
          Wifi settings
        </Button>
      </Card>
    </div>
  );
}
