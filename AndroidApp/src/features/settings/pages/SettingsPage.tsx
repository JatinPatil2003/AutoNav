import { useNavigate } from "react-router-dom";
import { Gauge, KeyRound, LogOut, Radio, ShieldCheck } from "lucide-react";
import { Avatar } from "../../../components/ui/Avatar";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { useSelectedRobot } from "../../../app/robotSelection/useSelectedRobot";
import { CurrentUser } from "../../auth/auth.types";
import { useLogoutMutation } from "../../auth/authApi";
import { useAuth } from "../../auth/useAuth";
import { useVelocitySettings } from "../useVelocitySettings";

type AuthProvider = CurrentUser["authProviders"][number];

const PROVIDER_META: Record<AuthProvider, { label: string; icon: typeof ShieldCheck }> = {
  google: { label: "Google", icon: ShieldCheck },
  local: { label: "Password", icon: KeyRound },
};

export function SettingsPage() {
  const { user } = useAuth();
  const { robots, selectedRobot } = useSelectedRobot();
  const { linearMultiplier, angularMultiplier, setLinearMultiplier, setAngularMultiplier } =
    useVelocitySettings();
  const [logout, { isLoading: isLoggingOut }] = useLogoutMutation();
  const navigate = useNavigate();

  async function handleLogout() {
    // Any control session that was open already closed itself when this
    // tab was navigated to — ControlPage's cleanup effect closes the WS
    // (and sends a release) on unmount, so there's nothing live to tear
    // down here. The logout mutation clears tokenStore regardless.
    await logout();
    navigate("/login", { replace: true });
  }

  return (
    <div className="flex flex-col gap-4 p-4 pb-safe-bottom">
      <Card className="glass-card">
        <div className="flex items-center gap-4">
          <Avatar name={user?.name} email={user?.email} avatarUrl={user?.avatarUrl} size={64} />
          <div className="min-w-0 flex-1">
            <p className="truncate text-base font-semibold text-slate-900 dark:text-slate-100">
              {user?.name ?? "—"}
            </p>
            <p className="truncate text-sm text-slate-500 dark:text-slate-400">{user?.email}</p>
            {user?.role === "admin" && (
              <span className="mt-1 inline-block rounded-full bg-brand-500/15 px-2 py-0.5 text-[11px] font-medium uppercase tracking-wide text-brand-600 dark:text-brand-400">
                Admin
              </span>
            )}
          </div>
        </div>

        {user && user.authProviders.length > 0 && (
          <div className="mt-4 flex flex-wrap gap-2 border-t border-slate-200/70 pt-3 dark:border-slate-700/60">
            {user.authProviders.map((provider: AuthProvider) => {
              const meta = PROVIDER_META[provider];
              const Icon = meta.icon;
              return (
                <span
                  key={provider}
                  className="inline-flex items-center gap-1.5 rounded-full bg-slate-100/80 px-2.5 py-1 text-xs font-medium text-slate-600 dark:bg-slate-700/60 dark:text-slate-300"
                >
                  <Icon className="h-3.5 w-3.5" />
                  {meta.label}
                </span>
              );
            })}
          </div>
        )}
      </Card>

      <Card>
        <div className="mb-1 flex items-center gap-2">
          <span className="flex h-8 w-8 items-center justify-center rounded-lg bg-brand-500/15">
            <Gauge className="h-4 w-4 text-brand-600 dark:text-brand-400" />
          </span>
          <h2 className="font-medium text-slate-900 dark:text-slate-100">Drive sensitivity</h2>
        </div>
        <p className="mb-4 mt-1 text-xs text-slate-500 dark:text-slate-400">
          Scales joystick output before it's sent — useful for driving more gently in tight spaces.
        </p>

        <label className="flex flex-col gap-2 text-sm text-slate-600 dark:text-slate-300">
          <span className="flex items-center justify-between">
            Linear
            <span className="font-semibold text-slate-900 dark:text-slate-100">
              {Math.round(linearMultiplier * 100)}%
            </span>
          </span>
          <input
            type="range"
            min={0}
            max={1}
            step={0.05}
            value={linearMultiplier}
            onChange={(e) => setLinearMultiplier(Number(e.target.value))}
            className="range-brand"
          />
        </label>

        <label className="mt-4 flex flex-col gap-2 text-sm text-slate-600 dark:text-slate-300">
          <span className="flex items-center justify-between">
            Angular
            <span className="font-semibold text-slate-900 dark:text-slate-100">
              {Math.round(angularMultiplier * 100)}%
            </span>
          </span>
          <input
            type="range"
            min={0}
            max={1}
            step={0.05}
            value={angularMultiplier}
            onChange={(e) => setAngularMultiplier(Number(e.target.value))}
            className="range-brand"
          />
        </label>
      </Card>

      {robots.length > 1 && (
        <Card>
          <div className="mb-2 flex items-center gap-2">
            <span className="flex h-8 w-8 items-center justify-center rounded-lg bg-brand-500/15">
              <Radio className="h-4 w-4 text-brand-600 dark:text-brand-400" />
            </span>
            <h2 className="font-medium text-slate-900 dark:text-slate-100">Robot</h2>
          </div>
          <p className="mb-3 text-sm text-slate-500 dark:text-slate-400">
            Operating {selectedRobot?.displayName ?? selectedRobot?.name ?? "—"}
          </p>
          <Button variant="secondary" className="w-full" onClick={() => navigate("/robot-select")}>
            Switch robot
          </Button>
        </Card>
      )}

      <Button variant="danger" isLoading={isLoggingOut} className="mt-2 gap-2" onClick={handleLogout}>
        <LogOut className="h-4 w-4" />
        Log out
      </Button>
    </div>
  );
}
