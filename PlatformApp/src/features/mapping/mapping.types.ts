import { SlamBackend } from "../telemetry/telemetry.types";

export interface StartMappingRequest {
  robotId: string;
  backend?: SlamBackend;
}

export interface CaptureWaypointRequest {
  robotId: string;
  name: string;
}

export interface CommitWaypointsRequest {
  robotId: string;
  mapName: string;
}

export interface SaveMapRequest {
  robotId: string;
  mapName: string;
}
