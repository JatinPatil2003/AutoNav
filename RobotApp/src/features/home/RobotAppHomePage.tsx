import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { UserAvatar } from "../../components/ui/UserAvatar";
import { SessionUser } from "../../api/hmiClient";
import { useTopic } from "../../api/topicsSocket";
import { maskEmail } from "../../utils/maskEmail";

interface RobotStatus {
  internetConnectivity: boolean;
}

interface RobotAppHomePageProps {
  user: SessionUser;
  onLogout: () => void;
  onOpenSettings: () => void;
}

/**
 * Minimal authenticated landing screen — a placeholder shell for this auth
 * milestone. Teleop/navigation/mapping controls are a future milestone,
 * once this screen exists to gate them behind a logged-in session.
 */
export function RobotAppHomePage({ user, onLogout, onOpenSettings }: RobotAppHomePageProps) {
  const robotStatus = useTopic<RobotStatus>("robotStatus");

  return (
    <div className="flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <Card className="w-full max-w-sm text-center">
        <UserAvatar
          name={user.name}
          avatarUrl={user.avatarUrl}
          className="mx-auto mb-3 h-16 w-16 text-2xl"
        />
        <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
          {user.name}
        </h1>
        <p className="mb-2 text-sm text-slate-500 dark:text-slate-400">{maskEmail(user.email)}</p>
        {robotStatus && (
          <p className="mb-6 flex items-center justify-center gap-1.5 text-xs text-slate-400 dark:text-slate-500">
            <span
              className={`h-2 w-2 rounded-full ${
                robotStatus.internetConnectivity ? "bg-emerald-500" : "bg-red-500"
              }`}
            />
            {robotStatus.internetConnectivity ? "Online" : "Offline"}
          </p>
        )}
        <div className="flex flex-col gap-2">
          <Button className="w-full" onClick={onOpenSettings}>
            Settings
          </Button>
          <Button variant="secondary" className="w-full" onClick={onLogout}>
            Switch user
          </Button>
          <Button variant="ghost" className="w-full" onClick={onLogout}>
            Log out
          </Button>
        </div>
      </Card>
    </div>
  );
}
