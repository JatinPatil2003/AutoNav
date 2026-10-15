import {
  BaseQueryApi,
  BaseQueryFn,
  FetchArgs,
  FetchBaseQueryError,
  createApi,
  fetchBaseQuery,
} from "@reduxjs/toolkit/query/react";

const rawBaseQuery = fetchBaseQuery({
  baseUrl: import.meta.env.VITE_API_BASE_URL,
  credentials: "include",
});

let refreshInFlight: Promise<boolean> | null = null;

async function refreshAccessToken(api: BaseQueryApi, extraOptions: object) {
  const refreshResult = await rawBaseQuery(
    { url: "/auth/refresh", method: "POST" },
    api,
    extraOptions,
  );
  return !refreshResult.error;
}

/**
 * Wraps fetchBaseQuery so a 401 triggers exactly one /auth/refresh attempt
 * (shared across concurrent requests via `refreshInFlight`) and retries the
 * original request once. The frontend never reads the JWTs themselves —
 * they live only in httpOnly cookies — so this is the only place that needs
 * to know refresh exists.
 */
const baseQueryWithReauth: BaseQueryFn<string | FetchArgs, unknown, FetchBaseQueryError> = async (
  args,
  api,
  extraOptions,
) => {
  let result = await rawBaseQuery(args, api, extraOptions);

  if (result.error?.status === 401) {
    refreshInFlight ??= refreshAccessToken(api, extraOptions).finally(() => {
      refreshInFlight = null;
    });

    const refreshed = await refreshInFlight;

    if (refreshed) {
      result = await rawBaseQuery(args, api, extraOptions);
    }
  }

  return result;
};

export const baseApi = createApi({
  reducerPath: "api",
  baseQuery: baseQueryWithReauth,
  tagTypes: [
    "Me",
    "RobotAccessRequest",
    "RobotOptions",
    "AdminRobotAccessRequests",
    "AdminRobots",
    "AdminUsers",
    "UserAssignments",
    "RobotAssignments",
  ],
  endpoints: () => ({}),
});
