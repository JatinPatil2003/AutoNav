import { Outlet } from "react-router-dom";
import { RobotSelectionProvider } from "../../../app/robotSelection/RobotSelectionProvider";
import { Sidebar } from "./Sidebar";
import { Topbar } from "./Topbar";

export function DashboardLayout() {
  return (
    <RobotSelectionProvider>
      <div className="flex h-screen bg-slate-50 dark:bg-slate-900">
        <Sidebar />
        <div className="flex flex-1 flex-col">
          <Topbar />
          <main className="flex-1 overflow-auto">
            <Outlet />
          </main>
        </div>
      </div>
    </RobotSelectionProvider>
  );
}
