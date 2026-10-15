import { baseApi } from "../../app/baseApi";
import { CurrentUser, LoginRequest, SignupRequest } from "./auth.types";

export const authApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    getMe: builder.query<CurrentUser, void>({
      query: () => "/users/me",
      transformResponse: (response: { user: CurrentUser }) => response.user,
      providesTags: ["Me"],
    }),
    signup: builder.mutation<CurrentUser, SignupRequest>({
      query: (body) => ({ url: "/auth/signup", method: "POST", body }),
      transformResponse: (response: { user: CurrentUser }) => response.user,
      invalidatesTags: ["Me"],
    }),
    login: builder.mutation<CurrentUser, LoginRequest>({
      query: (body) => ({ url: "/auth/login", method: "POST", body }),
      transformResponse: (response: { user: CurrentUser }) => response.user,
      invalidatesTags: ["Me"],
    }),
    logout: builder.mutation<void, void>({
      query: () => ({ url: "/auth/logout", method: "POST" }),
      invalidatesTags: ["Me", "RobotAccessRequest"],
    }),
    setPassword: builder.mutation<CurrentUser, { password: string }>({
      query: (body) => ({ url: "/auth/set-password", method: "POST", body }),
      transformResponse: (response: { user: CurrentUser }) => response.user,
      invalidatesTags: ["Me"],
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
  useLogoutMutation,
  useSetPasswordMutation,
  useForgotPasswordMutation,
  useVerifyOtpMutation,
  useResetPasswordMutation,
} = authApi;

export function googleLoginUrl(): string {
  return `${import.meta.env.VITE_API_BASE_URL}/auth/google`;
}
