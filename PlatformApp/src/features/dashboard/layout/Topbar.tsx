import { useNavigate } from "react-router-dom";
import { Button } from "../../../components/ui/Button";
import { ThemeToggle } from "../../../components/ui/ThemeToggle";
import { useAuth } from "../../auth/useAuth";
import { useLogoutMutation } from "../../auth/authApi";
import { RobotSummary } from "../../auth/auth.types";
import { useSelectedRobot } from "../../../app/robotSelection/useSelectedRobot";

export function Topbar() {
  const { user } = useAuth();
  const { robots, selectedRobotId, setSelectedRobotId } = useSelectedRobot();
  const [logout] = useLogoutMutation();
  const navigate = useNavigate();

  async function handleLogout() {
    await logout();
    navigate("/login", { replace: true });
  }

  return (
    <header className="glass-panel sticky top-0 z-10 m-3 mb-0 flex h-16 items-center justify-between rounded-2xl border px-6">
      <div className="text-sm text-slate-500 dark:text-slate-400">
        {robots.length === 0 ? (
          "No robot assigned"
        ) : robots.length === 1 ? (
          <span>
            Robot:{" "}
            <span className="font-medium text-slate-900 dark:text-slate-100">
              {robots[0].displayName ?? robots[0].name}
            </span>{" "}
            <span className="text-slate-400 dark:text-slate-500">({robots[0].vpnHostname})</span>
          </span>
        ) : (
          <div className="flex items-center gap-2">
            <span>Operating:</span>
            <select
              value={selectedRobotId ?? ""}
              onChange={(e) => setSelectedRobotId(e.target.value)}
              className="rounded-md border border-slate-300 bg-white px-2 py-1 text-sm font-medium text-slate-900 dark:border-slate-600 dark:bg-slate-900 dark:text-slate-100"
            >
              {robots.map((robot: RobotSummary) => (
                <option key={robot.id} value={robot.id}>
                  {robot.displayName ?? robot.name} ({robot.vpnHostname})
                </option>
              ))}
            </select>
          </div>
        )}
      </div>

      <div className="flex items-center gap-3">
        <span className="text-sm text-slate-700 dark:text-slate-300">{user?.name}</span>
        <ThemeToggle />
        <Button variant="ghost" onClick={handleLogout}>
          Log out
        </Button>
      </div>
    </header>
  );
}
