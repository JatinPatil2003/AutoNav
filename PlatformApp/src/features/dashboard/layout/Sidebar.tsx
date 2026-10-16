import { NavLink } from "react-router-dom";
import {
  BarChart3,
  BookOpen,
  LayoutDashboard,
  Map,
  Navigation2,
  Sparkles,
  Users,
  Bot,
  Inbox,
} from "lucide-react";
import { useAuth } from "../../auth/useAuth";

const navItems = [
  { to: "/", label: "Overview", icon: LayoutDashboard },
  { to: "/mapping", label: "Mapping", icon: Map },
  { to: "/navigation", label: "Navigation", icon: Navigation2 },
  { to: "/analytics", label: "Analytics", icon: BarChart3 },
  { to: "/docs", label: "Documentation", icon: BookOpen },
];

const adminItems = [
  { to: "/admin/requests", label: "Requests", icon: Inbox },
  { to: "/admin/users", label: "Users", icon: Users },
  { to: "/admin/robots", label: "Robots", icon: Bot },
];

function navLinkClass({ isActive }: { isActive: boolean }) {
  return `group flex items-center gap-2.5 rounded-xl px-3 py-2 text-sm font-medium transition-all ${
    isActive
      ? "bg-gradient-to-r from-brand-500/15 to-brand-500/5 text-brand-700 shadow-sm dark:from-brand-400/15 dark:to-brand-400/5 dark:text-brand-300"
      : "text-slate-600 hover:translate-x-0.5 hover:bg-white/60 hover:text-slate-900 dark:text-slate-300 dark:hover:bg-slate-800/60 dark:hover:text-slate-100"
  }`;
}

export function Sidebar() {
  const { isAdmin } = useAuth();

  return (
    <aside className="glass-panel relative z-10 m-3 mr-0 flex w-60 flex-col rounded-2xl border-r-0 p-4">
      <div className="mb-6 flex items-center gap-2 px-2">
        <span className="flex h-8 w-8 items-center justify-center rounded-lg bg-gradient-to-br from-brand-400 to-brand-600 shadow-md shadow-brand-900/20">
          <Sparkles className="h-4 w-4 text-white" />
        </span>
        <span className="text-lg font-bold tracking-tight text-slate-900 dark:text-slate-100">
          AutoNav
        </span>
      </div>
      <nav className="flex flex-col gap-1">
        {navItems.map((item) => (
          <NavLink key={item.to} to={item.to} end className={navLinkClass}>
            <item.icon className="h-4 w-4 flex-shrink-0 opacity-70" />
            {item.label}
          </NavLink>
        ))}
        {isAdmin && (
          <>
            <div className="my-3 flex items-center gap-2 px-3 text-[11px] font-semibold uppercase tracking-wider text-slate-400 dark:text-slate-500">
              <span className="h-px flex-1 bg-slate-200 dark:bg-slate-700" />
              Admin
              <span className="h-px flex-1 bg-slate-200 dark:bg-slate-700" />
            </div>
            {adminItems.map((item) => (
              <NavLink key={item.to} to={item.to} className={navLinkClass}>
                <item.icon className="h-4 w-4 flex-shrink-0 opacity-70" />
                {item.label}
              </NavLink>
            ))}
          </>
        )}
      </nav>
    </aside>
  );
}
