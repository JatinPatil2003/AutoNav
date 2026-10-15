export interface Pose {
  x: number;
  y: number;
  theta: number;
}

export interface Goal extends Pose {
  mapName: string;
  name: string;
}

export enum NavStatus {
  IDLE = "IDLE",
  ACTIVE = "ACTIVE",
  SUCCEEDED = "SUCCEEDED",
  CANCELED = "CANCELED",
  FAILED = "FAILED",
}
