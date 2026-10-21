export interface CategorizedGoals {
  localization: string[];
  charging: string[];
  standby: string[];
  goals: string[];
}

export interface GoalPose {
  mapName: string;
  name: string;
  x: number;
  y: number;
  theta: number;
}
