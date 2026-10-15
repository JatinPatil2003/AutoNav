import { NavLink } from "react-router-dom";
import { useAuth } from "../../auth/useAuth";

const navItems = [
  { to: "/", label: "Overview" },
  { to: "/mapping", label: "Mapping" },
  { to: "/navigation", label: "Navigation" },
  { to: "/analytics", label: "Analytics" },
  { to: "/docs", label: "Documentation" },
];

function navLinkClass({ isActive }: { isActive: boolean }) {
  return `relative rounded-md px-3 py-2 text-sm font-medium transition-colors ${
    isActive
      ? "bg-brand-50 text-brand-700 before:absolute before:-left-4 before:top-1.5 before:h-5 before:w-1 before:rounded-full before:bg-brand-600 dark:bg-brand-500/10 dark:text-brand-300 dark:before:bg-brand-400"
      : "text-slate-600 hover:bg-slate-100 dark:text-slate-300 dark:hover:bg-slate-800"
  }`;
}

export function Sidebar() {
  const { isAdmin } = useAuth();

  return (
    <aside className="flex w-56 flex-col border-r border-slate-200 bg-white p-4 pl-6 dark:border-slate-700 dark:bg-slate-800">
      <div className="mb-6 flex items-center gap-2 px-2">
        <span className="flex h-7 w-7 items-center justify-center rounded-md bg-brand-600 text-sm font-bold text-white">
          A
        </span>
        <span className="text-lg font-bold tracking-tight text-slate-900 dark:text-slate-100">
          AutoNav
        </span>
      </div>
      <nav className="flex flex-col gap-1">
        {navItems.map((item) => (
          <NavLink key={item.to} to={item.to} end className={navLinkClass}>
            {item.label}
          </NavLink>
        ))}
        {isAdmin && (
          <>
            <div className="my-2 h-px bg-slate-200 dark:bg-slate-700" />
            <NavLink to="/admin/requests" className={navLinkClass}>
              Admin: Requests
            </NavLink>
            <NavLink to="/admin/users" className={navLinkClass}>
              Admin: Users
            </NavLink>
            <NavLink to="/admin/robots" className={navLinkClass}>
              Admin: Robots
            </NavLink>
          </>
        )}
      </nav>
    </aside>
  );
}
