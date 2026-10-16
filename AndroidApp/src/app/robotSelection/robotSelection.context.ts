import { createContext } from "react";

export interface RobotSelectionContextValue {
  selectedRobotId: string | null;
  setSelectedRobotId: (robotId: string) => void;
  /** False until the persisted selection has been read from device storage. */
  isReady: boolean;
}

export const RobotSelectionContext = createContext<RobotSelectionContextValue | null>(null);
