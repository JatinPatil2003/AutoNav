import { baseApi } from "../../app/baseApi";
import { RobotAccessRequest } from "./robotAccess.types";

export const robotAccessApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    getMyRobotAccessRequest: builder.query<RobotAccessRequest | null, void>({
      query: () => "/robot-access-requests/me",
      transformResponse: (response: { request: RobotAccessRequest | null }) => response.request,
      providesTags: ["RobotAccessRequest"],
    }),
    createRobotAccessRequest: builder.mutation<RobotAccessRequest, void>({
      query: () => ({ url: "/robot-access-requests", method: "POST" }),
      transformResponse: (response: { request: RobotAccessRequest }) => response.request,
      invalidatesTags: ["RobotAccessRequest"],
    }),
  }),
});

export const { useGetMyRobotAccessRequestQuery, useCreateRobotAccessRequestMutation } =
  robotAccessApi;
