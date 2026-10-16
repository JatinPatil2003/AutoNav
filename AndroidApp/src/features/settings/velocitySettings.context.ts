import { createContext } from "react";

export interface VelocitySettingsContextValue {
  linearMultiplier: number;
  angularMultiplier: number;
  setLinearMultiplier: (value: number) => void;
  setAngularMultiplier: (value: number) => void;
}

export const VelocitySettingsContext = createContext<VelocitySettingsContextValue | null>(null);
