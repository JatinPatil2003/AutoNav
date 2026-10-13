import { Button } from "../../../components/ui/Button";
import { useAuth } from "../../auth/useAuth";
import { AdminUser } from "../admin.types";
import { useUpdateUserRoleMutation } from "../adminApi";

export function UsersTable({ users }: { users: AdminUser[] }) {
  const { user: currentUser } = useAuth();
  const [updateRole, { isLoading }] = useUpdateUserRoleMutation();

  if (users.length === 0) {
    return <p className="text-sm text-slate-500 dark:text-slate-400">No users yet.</p>;
  }

  return (
    <table className="w-full text-left text-sm text-slate-900 dark:text-slate-100">
      <thead>
        <tr className="border-b border-slate-200 text-slate-500 dark:border-slate-700 dark:text-slate-400">
          <th className="py-2 font-medium">Name</th>
          <th className="py-2 font-medium">Email</th>
          <th className="py-2 font-medium">Role</th>
          <th className="py-2 font-medium">Robot</th>
          <th className="py-2 font-medium text-right">Actions</th>
        </tr>
      </thead>
      <tbody>
        {users.map((user) => {
          const isSelf = user.id === currentUser?.id;
          const nextRole = user.role === "admin" ? "user" : "admin";

          return (
            <tr key={user.id} className="border-b border-slate-100 dark:border-slate-800">
              <td className="py-3">{user.name}</td>
              <td className="py-3 text-slate-500 dark:text-slate-400">{user.email}</td>
              <td className="py-3 capitalize">{user.role}</td>
              <td className="py-3 text-slate-500 dark:text-slate-400">
                {user.robot?.displayName ?? user.robot?.name ?? "—"}
              </td>
              <td className="py-3 text-right">
                <Button
                  variant="secondary"
                  isLoading={isLoading}
                  disabled={isSelf}
                  title={isSelf ? "You can't change your own role" : undefined}
                  onClick={() => updateRole({ id: user.id, role: nextRole })}
                >
                  {user.role === "admin" ? "Demote to user" : "Promote to admin"}
                </Button>
              </td>
            </tr>
          );
        })}
      </tbody>
    </table>
  );
}
