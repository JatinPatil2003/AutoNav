import { NavLink, Outlet } from "react-router-dom";
import { Gauge, LineChart, Settings as SettingsIcon } from "lucide-react";
import { VelocitySettingsProvider } from "../settings/VelocitySettingsProvider";

const TABS = [
  { to: "/control", label: "Control", icon: Gauge },
  { to: "/analytics", label: "Analytics", icon: LineChart },
  { to: "/settings", label: "Settings", icon: SettingsIcon },
];

/**
 * Root authenticated shell: a floating bottom tab bar (Control | Analytics
 * | Settings) over a routed <Outlet/>, matching the plan's "HomeShell" —
 * mobile-first bottom nav rather than PlatformApp's desktop sidebar.
 */
export function HomeShell() {
  return (
    <VelocitySettingsProvider>
      <div className="app-canvas flex h-full flex-col">
        <main className="flex-1 overflow-auto">
          <Outlet />
        </main>
        <nav className="glass-card mx-3 mb-3 flex gap-1 rounded-2xl p-1.5 shadow-lg shadow-slate-900/10">
          {TABS.map((tab) => (
            <NavLink
              key={tab.to}
              to={tab.to}
              className={({ isActive }) =>
                `flex flex-1 flex-col items-center gap-1 rounded-xl py-2 text-xs font-medium transition-colors ${
                  isActive
                    ? "bg-brand-500/15 text-brand-600 dark:bg-brand-400/15 dark:text-brand-400"
                    : "text-slate-400 hover:text-slate-500 dark:text-slate-500 dark:hover:text-slate-400"
                }`
              }
            >
              <tab.icon className="h-5 w-5" />
              {tab.label}
            </NavLink>
          ))}
        </nav>
      </div>
    </VelocitySettingsProvider>
  );
}
