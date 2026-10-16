import { PropsWithChildren, useCallback, useEffect, useState } from "react";
import { readPreference, writePreference } from "../../native/preferences";
import { VelocitySettingsContext } from "./velocitySettings.context";

const STORAGE_KEY = "autonav-velocity-multiplier";
const DEFAULT_MULTIPLIER = 1;

function clampMultiplier(value: number): number {
  if (!Number.isFinite(value)) return DEFAULT_MULTIPLIER;
  return Math.min(1, Math.max(0, value));
}

/**
 * Client-side-only velocity scaling applied to joystick output before it's
 * sent over the control WS — net-new, no prior art elsewhere in this repo.
 * Range is always 0–1 (it only ever *reduces* what the stick reports, never
 * amplifies it); the backend's own TELEOP_MAX_LINEAR/ANGULAR clamps remain
 * the real ceiling. Persisted via Capacitor Preferences since it's a
 * harmless per-device UI preference, not a credential.
 */
export function VelocitySettingsProvider({ children }: PropsWithChildren) {
  const [linearMultiplier, setLinearMultiplierState] = useState(DEFAULT_MULTIPLIER);
  const [angularMultiplier, setAngularMultiplierState] = useState(DEFAULT_MULTIPLIER);

  useEffect(() => {
    let cancelled = false;

    async function load() {
      const stored = await readPreference(STORAGE_KEY);
      if (cancelled || !stored) return;
      try {
        const parsed = JSON.parse(stored) as { linear?: number; angular?: number };
        if (typeof parsed.linear === "number") setLinearMultiplierState(clampMultiplier(parsed.linear));
        if (typeof parsed.angular === "number") setAngularMultiplierState(clampMultiplier(parsed.angular));
      } catch {
        // Corrupted/old stored value — fall back to the defaults already set.
      }
    }

    void load();
    return () => {
      cancelled = true;
    };
  }, []);

  const persist = useCallback((linear: number, angular: number) => {
    void writePreference(STORAGE_KEY, JSON.stringify({ linear, angular }));
  }, []);

  const setLinearMultiplier = useCallback(
    (value: number) => {
      const clamped = clampMultiplier(value);
      setLinearMultiplierState(clamped);
      persist(clamped, angularMultiplier);
    },
    [angularMultiplier, persist],
  );

  const setAngularMultiplier = useCallback(
    (value: number) => {
      const clamped = clampMultiplier(value);
      setAngularMultiplierState(clamped);
      persist(linearMultiplier, clamped);
    },
    [linearMultiplier, persist],
  );

  return (
    <VelocitySettingsContext.Provider
      value={{ linearMultiplier, angularMultiplier, setLinearMultiplier, setAngularMultiplier }}
    >
      {children}
    </VelocitySettingsContext.Provider>
  );
}
