import { baseApi } from "../../app/baseApi";
import { CaptureWaypointRequest, CommitWaypointsRequest, SaveMapRequest, StartMappingRequest } from "./mapping.types";

/**
 * Request/response mapping commands, proxied by PlatformServer straight
 * through to the robot's own RobotServer REST API over the VPN (see
 * PlatformServer's robot-proxy module) — distinct from the live map/pose
 * stream, which comes over /ws/telemetry (see features/telemetry).
 */
export const mappingApi = baseApi.injectEndpoints({
  endpoints: (builder) => ({
    startMapping: builder.mutation<{ status: string }, StartMappingRequest>({
      query: ({ robotId, backend }) => ({
        url: `/robots/${robotId}/mapping/start`,
        method: "POST",
        body: backend ? { backend } : {},
      }),
    }),
    stopMapping: builder.mutation<{ status: string }, { robotId: string }>({
      query: ({ robotId }) => ({ url: `/robots/${robotId}/mapping/stop`, method: "POST" }),
    }),
    captureWaypoint: builder.mutation<{ status: string }, CaptureWaypointRequest>({
      query: ({ robotId, name }) => ({ url: `/robots/${robotId}/mapping/waypoints`, method: "POST", body: { name } }),
    }),
    commitWaypoints: builder.mutation<{ status: string; processed: number }, CommitWaypointsRequest>({
      query: ({ robotId, mapName }) => ({
        url: `/robots/${robotId}/mapping/waypoints/commit`,
        method: "POST",
        body: { mapName },
      }),
    }),
    saveMap: builder.mutation<{ status: string }, SaveMapRequest>({
      query: ({ robotId, mapName }) => ({ url: `/robots/${robotId}/mapping/save`, method: "POST", body: { mapName } }),
      invalidatesTags: ["RobotMaps"],
    }),
  }),
});

export const {
  useStartMappingMutation,
  useStopMappingMutation,
  useCaptureWaypointMutation,
  useCommitWaypointsMutation,
  useSaveMapMutation,
} = mappingApi;
