import { createContext } from "react";

export interface RobotSelectionContextValue {
  selectedRobotId: string | null;
  setSelectedRobotId: (robotId: string) => void;
}

export const RobotSelectionContext = createContext<RobotSelectionContextValue | null>(null);
