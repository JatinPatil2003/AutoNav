import { useState } from "react";
import { Button } from "../../../components/ui/Button";
import { RobotAccessRequest } from "../admin.types";
import { useRejectRequestMutation } from "../adminApi";
import { ApproveRequestDialog } from "./ApproveRequestDialog";

export function RequestsTable({ requests }: { requests: RobotAccessRequest[] }) {
  const [approvingRequest, setApprovingRequest] = useState<RobotAccessRequest | null>(null);
  const [reject, { isLoading: isRejecting }] = useRejectRequestMutation();

  if (requests.length === 0) {
    return <p className="text-sm text-slate-500 dark:text-slate-400">No pending requests.</p>;
  }

  return (
    <>
      <table className="w-full text-left text-sm text-slate-900 dark:text-slate-100">
        <thead>
          <tr className="border-b border-slate-200 text-slate-500 dark:border-slate-700 dark:text-slate-400">
            <th className="py-2 font-medium">Requester</th>
            <th className="py-2 font-medium">Email</th>
            <th className="py-2 font-medium">Requested robot</th>
            <th className="py-2 font-medium">Requested</th>
            <th className="py-2 font-medium text-right">Actions</th>
          </tr>
        </thead>
        <tbody>
          {requests.map((request) => (
            <tr key={request.id} className="border-b border-slate-100 dark:border-slate-800">
              <td className="py-3">{request.requestedBy?.name ?? "—"}</td>
              <td className="py-3 text-slate-500 dark:text-slate-400">
                {request.requestedBy?.email ?? "—"}
              </td>
              <td className="py-3 text-slate-500 dark:text-slate-400">
                {request.robot?.displayName ?? request.robot?.name ?? "—"}
              </td>
              <td className="py-3 text-slate-500 dark:text-slate-400">
                {new Date(request.createdAt).toLocaleString()}
              </td>
              <td className="py-3 text-right">
                <div className="flex justify-end gap-2">
                  <Button
                    variant="secondary"
                    onClick={() => setApprovingRequest(request)}
                  >
                    Approve
                  </Button>
                  <Button
                    variant="danger"
                    isLoading={isRejecting}
                    onClick={() => reject({ id: request.id })}
                  >
                    Reject
                  </Button>
                </div>
              </td>
            </tr>
          ))}
        </tbody>
      </table>

      {approvingRequest && (
        <ApproveRequestDialog request={approvingRequest} onClose={() => setApprovingRequest(null)} />
      )}
    </>
  );
}
