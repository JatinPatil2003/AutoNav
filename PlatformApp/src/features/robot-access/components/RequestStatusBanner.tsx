import { RobotAccessRequest } from "../robotAccess.types";

export function RequestStatusBanner({ request }: { request: RobotAccessRequest }) {
  if (request.status === "pending") {
    return (
      <div className="rounded-md bg-amber-50 px-4 py-3 text-sm text-amber-800 dark:bg-amber-900/30 dark:text-amber-300">
        Your request is pending review by an administrator.
      </div>
    );
  }

  if (request.status === "rejected") {
    return (
      <div className="rounded-md bg-red-50 px-4 py-3 text-sm text-red-800 dark:bg-red-900/30 dark:text-red-300">
        Your previous request was rejected
        {request.rejectionReason ? `: ${request.rejectionReason}` : "."} You can request access
        again below.
      </div>
    );
  }

  return null;
}
