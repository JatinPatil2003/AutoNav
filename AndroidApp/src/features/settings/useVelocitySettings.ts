import { useContext } from "react";
import { VelocitySettingsContext } from "./velocitySettings.context";

export function useVelocitySettings() {
  const ctx = useContext(VelocitySettingsContext);
  if (!ctx) {
    throw new Error("useVelocitySettings must be used within a VelocitySettingsProvider");
  }
  return ctx;
}
