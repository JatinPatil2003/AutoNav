import { Card } from "../../../components/ui/Card";
import { AddRobotForm } from "../components/AddRobotForm";
import { RobotsList } from "../components/RobotsList";

export function AdminRobotsPage() {
  return (
    <div className="flex flex-col gap-6 p-6">
      <h1 className="text-xl font-semibold text-slate-900 dark:text-slate-100">Robots</h1>

      <div className="grid gap-6 md:grid-cols-2">
        <div>
          <h2 className="mb-4 text-lg font-semibold text-slate-900 dark:text-slate-100">Add a robot</h2>
          <Card>
            <AddRobotForm />
          </Card>
        </div>
        <div>
          <h2 className="mb-4 text-lg font-semibold text-slate-900 dark:text-slate-100">All robots</h2>
          <Card>
            <RobotsList />
          </Card>
        </div>
      </div>
    </div>
  );
}
