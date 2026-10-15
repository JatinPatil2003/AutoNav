import { Card } from "../../../components/ui/Card";
import { FullPageSpinner } from "../../../components/ui/Spinner";
import { RequestsTable } from "../components/RequestsTable";
import { useListPendingRequestsQuery } from "../adminApi";

export function AdminRequestsPage() {
  const { data: requests, isLoading } = useListPendingRequestsQuery();

  if (isLoading) return <FullPageSpinner />;

  return (
    <div className="p-6">
      <h1 className="mb-4 text-xl font-semibold text-slate-900 dark:text-slate-100">Pending robot access requests</h1>
      <Card>
        <RequestsTable requests={requests ?? []} />
      </Card>
    </div>
  );
}
