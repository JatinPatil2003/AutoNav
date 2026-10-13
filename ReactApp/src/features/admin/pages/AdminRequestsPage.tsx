import { Card } from "../../../components/ui/Card";
import { FullPageSpinner } from "../../../components/ui/Spinner";
import { RequestsTable } from "../components/RequestsTable";
import { AddRobotForm } from "../components/AddRobotForm";
import { RobotsList } from "../components/RobotsList";
import { useListPendingRequestsQuery } from "../adminApi";

export function AdminRequestsPage() {
  const { data: requests, isLoading } = useListPendingRequestsQuery();

  if (isLoading) return <FullPageSpinner />;

  return (
    <div className="flex flex-col gap-6 p-6">
      <div>
        <h1 className="mb-4 text-xl font-semibold text-slate-900 dark:text-slate-100">Pending robot access requests</h1>
        <Card>
          <RequestsTable requests={requests ?? []} />
        </Card>
      </div>

      <div className="grid gap-6 md:grid-cols-2">
        <div>
          <h2 className="mb-4 text-lg font-semibold text-slate-900 dark:text-slate-100">Add a robot</h2>
          <Card>
            <AddRobotForm />
          </Card>
        </div>
        <div>
          <h2 className="mb-4 text-lg font-semibold text-slate-900 dark:text-slate-100">Robots</h2>
          <Card>
            <RobotsList />
          </Card>
        </div>
      </div>
    </div>
  );
}
