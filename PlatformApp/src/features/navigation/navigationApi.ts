import { baseApi } from "../../app/baseApi";
import { CategorizedGoals, GoalPose } from "./navigation.types";

export const navigationApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    listMaps: builder.query<{ maps: string[] }, { robotId: string }>({
      query: ({ robotId }) => `/robots/${robotId}/navigation/maps`,
      providesTags: ["RobotMaps"],
    }),
    selectMap: builder.mutation<{ status: string }, { robotId: string; mapName: string }>({
      query: ({ robotId, mapName }) => ({
        url: `/robots/${robotId}/navigation/maps/${encodeURIComponent(mapName)}/select`,
        method: "POST",
      }),
    }),
    listGoals: builder.query<CategorizedGoals, { robotId: string; mapName: string }>({
      query: ({ robotId, mapName }) => `/robots/${robotId}/navigation/maps/${encodeURIComponent(mapName)}/goals`,
      providesTags: ["RobotGoals"],
    }),
    saveGoal: builder.mutation<{ status: string }, { robotId: string } & GoalPose>({
      query: ({ robotId, ...goal }) => ({ url: `/robots/${robotId}/navigation/goals`, method: "POST", body: goal }),
      invalidatesTags: ["RobotGoals"],
    }),
    startGoal: builder.mutation<{ status: string }, { robotId: string; mapName: string; name: string }>({
      query: ({ robotId, mapName, name }) => ({
        url: `/robots/${robotId}/navigation/goal/start`,
        method: "POST",
        body: { mapName, name },
      }),
    }),
    cancelGoal: builder.mutation<{ status: string }, { robotId: string }>({
      query: ({ robotId }) => ({ url: `/robots/${robotId}/navigation/goal/cancel`, method: "POST" }),
    }),
  }),
});

export const {
  useListMapsQuery,
  useSelectMapMutation,
  useListGoalsQuery,
  useSaveGoalMutation,
  useStartGoalMutation,
  useCancelGoalMutation,
} = navigationApi;
