import { FetchBaseQueryError } from "@reduxjs/toolkit/query/react";
import { SerializedError } from "@reduxjs/toolkit";

export function getApiErrorMessage(error: FetchBaseQueryError | SerializedError | undefined): string {
  if (!error) return "Something went wrong";

  if ("status" in error) {
    const data = error.data as { message?: string } | undefined;
    return data?.message ?? "Something went wrong";
  }

  return error.message ?? "Something went wrong";
}
