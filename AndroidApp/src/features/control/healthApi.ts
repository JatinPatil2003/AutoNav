import { baseApi } from "../../app/baseApi";
import { RobotHealth } from "./control.types";

export const healthApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    // Deliberately no providesTags/invalidatesTags — this is a one-shot
    // liveness probe polled on an interval by the caller, not a resource
    // whose cache should be invalidated by some other mutation.
    getRobotHealth: builder.query<RobotHealth, string>({
      query: (robotId) => `/robots/${robotId}/health`,
      keepUnusedDataFor: 0,
    }),
  }),
});

export const { useGetRobotHealthQuery } = healthApi;
