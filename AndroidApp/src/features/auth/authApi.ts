import { baseApi } from "../../app/baseApi";
import { TokenPair, tokenStore } from "../../app/tokenStore";
import { CurrentUser, GoogleTokenRequest, LoginRequest, SignupRequest } from "./auth.types";

interface AuthResponse {
  user: CurrentUser;
  tokens?: TokenPair;
}

/**
 * Every endpoint that can mint a fresh token pair (signup/login/google)
 * persists it as a side effect of transformResponse — this is the one
 * spot in the app allowed to reach into tokenStore directly for a *new*
 * pair, since baseApi's own refresh flow only ever rotates an existing one.
 */
function persistTokensAndReturnUser(response: AuthResponse): CurrentUser {
  if (response.tokens) {
    tokenStore.setAccessToken(response.tokens.accessToken);
    void tokenStore.setRefreshToken(response.tokens.refreshToken);
  }
  return response.user;
}

export const authApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    getMe: builder.query<CurrentUser, void>({
      query: () => "/users/me",
      transformResponse: (response: { user: CurrentUser }) => response.user,
      providesTags: ["Me"],
    }),
    signup: builder.mutation<CurrentUser, SignupRequest>({
      query: (body) => ({ url: "/auth/signup", method: "POST", body }),
      transformResponse: persistTokensAndReturnUser,
      invalidatesTags: ["Me"],
    }),
    login: builder.mutation<CurrentUser, LoginRequest>({
      query: (body) => ({ url: "/auth/login", method: "POST", body }),
      transformResponse: persistTokensAndReturnUser,
      invalidatesTags: ["Me"],
    }),
    googleToken: builder.mutation<CurrentUser, GoogleTokenRequest>({
      query: (body) => ({ url: "/auth/google/token", method: "POST", body }),
      transformResponse: persistTokensAndReturnUser,
      invalidatesTags: ["Me"],
    }),
    logout: builder.mutation<void, void>({
      // A native queryFn (rather than a plain `query`) because logout needs
      // to read the refresh token out of secure storage first (there's no
      // cookie to fall back on) and must clear all local credentials
      // afterwards regardless of whether the server call succeeds.
      async queryFn(_arg, _api, _extraOptions, baseQuery) {
        const refreshToken = await tokenStore.getRefreshToken();
        const result = await baseQuery({
          url: "/auth/logout",
          method: "POST",
          body: refreshToken ? { refreshToken } : undefined,
        });
        await tokenStore.clear();
        return result.error ? { error: result.error } : { data: undefined };
      },
      invalidatesTags: ["Me", "RobotAccessRequest"],
    }),
    forgotPassword: builder.mutation<void, { email: string }>({
      query: (body) => ({ url: "/auth/forgot-password", method: "POST", body }),
    }),
    verifyOtp: builder.mutation<{ resetToken: string }, { email: string; otp: string }>({
      query: (body) => ({ url: "/auth/verify-otp", method: "POST", body }),
    }),
    resetPassword: builder.mutation<void, { resetToken: string; newPassword: string }>({
      query: (body) => ({ url: "/auth/reset-password", method: "POST", body }),
    }),
  }),
});

export const {
  useGetMeQuery,
  useSignupMutation,
  useLoginMutation,
  useGoogleTokenMutation,
  useLogoutMutation,
  useForgotPasswordMutation,
  useVerifyOtpMutation,
  useResetPasswordMutation,
} = authApi;
