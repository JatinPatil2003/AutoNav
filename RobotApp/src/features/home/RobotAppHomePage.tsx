import { LogOut, Settings, Users, Wifi, WifiOff } from "lucide-react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { AppShell } from "../../components/ui/AppShell";
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
  const online = robotStatus?.internetConnectivity ?? false;

  return (
    <AppShell>
      <Card className="text-center">
        <div className="relative mx-auto mb-4 w-fit">
          <UserAvatar name={user.name} avatarUrl={user.avatarUrl} className="h-20 w-20 text-3xl" glow />
          <span
            className={`absolute -bottom-0.5 -right-0.5 flex h-5 w-5 items-center justify-center rounded-full border-2 border-white dark:border-slate-900 ${
              online ? "bg-emerald-500" : "bg-red-500"
            }`}
          >
            <span className={`h-2 w-2 rounded-full bg-white/90 ${online ? "animate-pulse" : ""}`} />
          </span>
        </div>

        <h1 className="mb-1 text-2xl font-bold tracking-tight text-slate-900 dark:text-slate-50">
          Welcome back, {user.name.split(" ")[0]}
        </h1>
        <p className="mb-3 text-sm text-slate-500 dark:text-slate-400">{maskEmail(user.email)}</p>

        {robotStatus && (
          <span
            className={`mb-6 inline-flex items-center gap-1.5 rounded-full px-3 py-1 text-xs font-semibold ${
              online
                ? "bg-emerald-50 text-emerald-700 dark:bg-emerald-900/20 dark:text-emerald-300"
                : "bg-red-50 text-red-600 dark:bg-red-900/20 dark:text-red-400"
            }`}
          >
            {online ? <Wifi className="h-3.5 w-3.5" /> : <WifiOff className="h-3.5 w-3.5" />}
            {online ? "Online" : "Offline"}
          </span>
        )}

        <div className="mb-6 rounded-xl border border-dashed border-slate-200 bg-slate-50/60 px-4 py-6 text-xs text-slate-400 dark:border-slate-700 dark:bg-slate-800/30 dark:text-slate-500">
          Teleop, navigation &amp; mapping controls land here in a future milestone.
        </div>

        <div className="flex flex-col gap-2">
          <Button className="w-full" onClick={onOpenSettings}>
            <Settings className="h-4 w-4" /> Settings
          </Button>
          <Button variant="secondary" className="w-full" onClick={onLogout}>
            <Users className="h-4 w-4" /> Switch user
          </Button>
          <Button variant="ghost" className="w-full" onClick={onLogout}>
            <LogOut className="h-4 w-4" /> Log out
          </Button>
        </div>
      </Card>
    </AppShell>
  );
}
