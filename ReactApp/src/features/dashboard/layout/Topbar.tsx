import { useNavigate } from "react-router-dom";
import { Button } from "../../../components/ui/Button";
import { ThemeToggle } from "../../../components/ui/ThemeToggle";
import { useAuth } from "../../auth/useAuth";
import { useLogoutMutation } from "../../auth/authApi";

export function Topbar() {
  const { user } = useAuth();
  const [logout] = useLogoutMutation();
  const navigate = useNavigate();

  async function handleLogout() {
    await logout();
    navigate("/login", { replace: true });
  }

  return (
    <header className="sticky top-0 z-10 flex h-16 items-center justify-between border-b border-slate-200 bg-white/80 px-6 backdrop-blur dark:border-slate-700 dark:bg-slate-800/80">
      <div className="text-sm text-slate-500 dark:text-slate-400">
        {user?.robot ? (
          <span>
            Robot:{" "}
            <span className="font-medium text-slate-900 dark:text-slate-100">
              {user.robot.displayName ?? user.robot.name}
            </span>{" "}
            <span className="text-slate-400 dark:text-slate-500">({user.robot.vpnHostname})</span>
          </span>
        ) : (
          "No robot assigned"
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
