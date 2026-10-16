import { baseApi } from "../../app/baseApi";
import { RobotAccessRequest, RobotOption } from "./robotAccess.types";

export const robotAccessApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    listRobotOptions: builder.query<RobotOption[], void>({
      query: () => "/robots",
      transformResponse: (response: { robots: RobotOption[] }) => response.robots,
      providesTags: ["RobotOptions"],
    }),
    listMyRobotAccessRequests: builder.query<RobotAccessRequest[], void>({
      query: () => "/robot-access-requests/mine",
      transformResponse: (response: { requests: RobotAccessRequest[] }) => response.requests,
      providesTags: ["RobotAccessRequest"],
    }),
    createRobotAccessRequest: builder.mutation<RobotAccessRequest, { robotId: string }>({
      query: (body) => ({ url: "/robot-access-requests", method: "POST", body }),
      transformResponse: (response: { request: RobotAccessRequest }) => response.request,
      invalidatesTags: ["RobotAccessRequest"],
    }),
  }),
});

export const {
  useListRobotOptionsQuery,
  useListMyRobotAccessRequestsQuery,
  useCreateRobotAccessRequestMutation,
} = robotAccessApi;
