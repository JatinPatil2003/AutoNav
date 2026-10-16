import { Outlet } from "react-router-dom";
import { RobotSelectionProvider } from "../../../app/robotSelection/RobotSelectionProvider";
import { Sidebar } from "./Sidebar";
import { Topbar } from "./Topbar";

export function DashboardLayout() {
  return (
    <RobotSelectionProvider>
      <div className="app-canvas flex h-screen">
        <Sidebar />
        <div className="flex flex-1 flex-col overflow-hidden">
          <Topbar />
          <main className="flex-1 overflow-auto p-3">
            <Outlet />
          </main>
        </div>
      </div>
    </RobotSelectionProvider>
  );
}
