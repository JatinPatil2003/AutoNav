import { Card } from "../../../components/ui/Card";
import { FullPageSpinner } from "../../../components/ui/Spinner";
import { UsersTable } from "../components/UsersTable";
import { useListUsersQuery } from "../adminApi";

export function AdminUsersPage() {
  const { data: users, isLoading } = useListUsersQuery();

  if (isLoading) return <FullPageSpinner />;

  return (
    <div className="p-6">
      <h1 className="mb-4 text-xl font-semibold text-slate-900 dark:text-slate-100">Users</h1>
      <Card>
        <UsersTable users={users ?? []} />
      </Card>
    </div>
  );
}
