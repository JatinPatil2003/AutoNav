import { baseApi } from "../../app/baseApi";
import { RobotAccessRequest, RobotOption } from "./robotAccess.types";

export const robotAccessApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    lookupRobot: builder.query<RobotOption | null, string>({
      query: (robotId) => `/robots/lookup/${encodeURIComponent(robotId)}`,
      transformResponse: (response: { robot: RobotOption | null }) => response.robot,
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
  useLazyLookupRobotQuery,
  useListMyRobotAccessRequestsQuery,
  useCreateRobotAccessRequestMutation,
} = robotAccessApi;
