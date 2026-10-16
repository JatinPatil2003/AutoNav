import {
  BaseQueryApi,
  BaseQueryFn,
  FetchArgs,
  FetchBaseQueryError,
  createApi,
  fetchBaseQuery,
} from "@reduxjs/toolkit/query/react";
import { TokenPair, tokenStore } from "./tokenStore";

const rawBaseQuery = fetchBaseQuery({
  baseUrl: import.meta.env.VITE_API_BASE_URL,
  prepareHeaders: (headers) => {
    const token = tokenStore.getAccessToken();
    if (token) headers.set("Authorization", `Bearer ${token}`);
    // Tells PlatformServer to return tokens in the JSON body (see
    // auth.controller.ts's isMobileClient/buildAuthResponse) instead of
    // relying solely on Set-Cookie — a Capacitor WebView's cross-origin
    // cookie jar isn't something this app can depend on.
    headers.set("X-Client", "mobile");
    return headers;
  },
});

let refreshInFlight: Promise<boolean> | null = null;

async function refreshAccessToken(api: BaseQueryApi, extraOptions: object): Promise<boolean> {
  const refreshToken = await tokenStore.getRefreshToken();
  if (!refreshToken) return false;

  const result = await rawBaseQuery(
    { url: "/auth/refresh", method: "POST", body: { refreshToken } },
    api,
    extraOptions,
  );

  const data = result.data as { tokens?: TokenPair } | undefined;
  if (result.error || !data?.tokens) {
    await tokenStore.clear();
    return false;
  }

  await tokenStore.setTokenPair(data.tokens);
  return true;
}

/**
 * Mirrors PlatformApp's baseQueryWithReauth single-flight-refresh pattern
 * (a 401 triggers exactly one /auth/refresh attempt, shared across
 * concurrent requests via `refreshInFlight`, then the original request is
 * retried once) with one divergence: the refresh call carries the refresh
 * token in the request body — read from secure storage — instead of
 * relying on a cookie, since this app never has a reliable cookie jar.
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
  tagTypes: ["Me", "RobotAccessRequest", "RobotOptions"],
  endpoints: () => ({}),
});
