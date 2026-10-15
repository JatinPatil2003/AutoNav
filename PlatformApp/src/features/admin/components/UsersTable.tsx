import { useState } from "react";
import { Button } from "../../../components/ui/Button";
import { getApiErrorMessage } from "../../../app/apiError";
import { useAuth } from "../../auth/useAuth";
import { AdminUser } from "../admin.types";
import { useDeleteUserMutation, useUpdateUserRoleMutation } from "../adminApi";
import { ManageUserRobotsDialog } from "./ManageUserRobotsDialog";

export function UsersTable({ users }: { users: AdminUser[] }) {
  const { user: currentUser } = useAuth();
  const [updateRole, { isLoading }] = useUpdateUserRoleMutation();
  const [deleteUser, { isLoading: isDeleting, error: deleteError }] = useDeleteUserMutation();
  const [managingUser, setManagingUser] = useState<AdminUser | null>(null);

  async function handleDelete(user: AdminUser) {
    const confirmed = window.confirm(
      `Permanently delete "${user.name}" (${user.email})? This also removes all of their robot assignments and access requests.`,
    );
    if (!confirmed) return;
    await deleteUser(user.id).unwrap();
  }

  if (users.length === 0) {
    return <p className="text-sm text-slate-500 dark:text-slate-400">No users yet.</p>;
  }

  return (
    <>
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
                {user.robots.length > 0
                  ? user.robots.map((r) => r.displayName ?? r.name).join(", ")
                  : "—"}
              </td>
              <td className="py-3 text-right">
                <div className="flex justify-end gap-2">
                  <Button variant="secondary" onClick={() => setManagingUser(user)}>
                    Manage robots
                  </Button>
                  <Button
                    variant="secondary"
                    isLoading={isLoading}
                    disabled={isSelf}
                    title={isSelf ? "You can't change your own role" : undefined}
                    onClick={() => updateRole({ id: user.id, role: nextRole })}
                  >
                    {user.role === "admin" ? "Demote to user" : "Promote to admin"}
                  </Button>
                  <Button
                    variant="danger"
                    isLoading={isDeleting}
                    disabled={isSelf}
                    title={isSelf ? "You can't delete your own account" : undefined}
                    onClick={() => handleDelete(user)}
                  >
                    Delete
                  </Button>
                </div>
              </td>
            </tr>
          );
        })}
      </tbody>
    </table>
    {deleteError && (
      <p className="mt-2 text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(deleteError)}</p>
    )}
    {managingUser && (
      <ManageUserRobotsDialog user={managingUser} onClose={() => setManagingUser(null)} />
    )}
    </>
  );
}
