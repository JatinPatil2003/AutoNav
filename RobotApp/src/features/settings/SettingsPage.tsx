import { useState } from "react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { getApiErrorMessage } from "../../api/apiError";
import { LED_PATTERN_IDLE, LED_PATTERN_OFF, firmwareClient } from "../../api/firmwareClient";

interface SettingsPageProps {
  onBack: () => void;
}

/**
 * Display controls for this robot's LED strip, via RobotServer's
 * /api/firmware/led/* (the ESP32 bridge over ROS2 — see firmware.service.ts).
 * There's no "get current state" endpoint on the firmware bridge, so this
 * page tracks light/brightness optimistically rather than reading them back.
 */
export function SettingsPage({ onBack }: SettingsPageProps) {
  const [lightOn, setLightOn] = useState(false);
  const [brightness, setBrightness] = useState(80);
  const [isLightLoading, setIsLightLoading] = useState(false);
  const [isBrightnessLoading, setIsBrightnessLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function handleToggleLight() {
    const nextOn = !lightOn;
    setError(null);
    setIsLightLoading(true);
    try {
      const result = await firmwareClient.setLedPattern(nextOn ? LED_PATTERN_IDLE : LED_PATTERN_OFF);
      if (!result.success) throw new Error(result.message);
      setLightOn(nextOn);
    } catch (err) {
      setError(getApiErrorMessage(err));
    } finally {
      setIsLightLoading(false);
    }
  }

  async function handleBrightnessChange(level: number) {
    setBrightness(level);
    setError(null);
    setIsBrightnessLoading(true);
    try {
      // Firmware brightness is 0-255; the slider is 0-100%.
      const result = await firmwareClient.setBrightness(Math.round((level / 100) * 255));
      if (!result.success) throw new Error(result.message);
    } catch (err) {
      setError(getApiErrorMessage(err));
    } finally {
      setIsBrightnessLoading(false);
    }
  }

  return (
    <div className="flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <Card className="w-full max-w-sm">
        <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">Settings</h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">LED controls for this robot.</p>

        {error && (
          <p className="mb-4 text-sm text-red-600 dark:text-red-400">{error}</p>
        )}

        <div className="mb-6 flex items-center justify-between rounded-md border border-slate-200 px-4 py-3 dark:border-slate-700">
          <span className="text-sm font-medium text-slate-900 dark:text-slate-100">Light</span>
          <Button variant={lightOn ? "primary" : "secondary"} isLoading={isLightLoading} onClick={handleToggleLight}>
            {lightOn ? "On" : "Off"}
          </Button>
        </div>

        <div className="mb-6 rounded-md border border-slate-200 px-4 py-3 dark:border-slate-700">
          <div className="mb-2 flex items-center justify-between">
            <span className="text-sm font-medium text-slate-900 dark:text-slate-100">Brightness</span>
            <span className="text-sm text-slate-500 dark:text-slate-400">{brightness}%</span>
          </div>
          <input
            type="range"
            min={0}
            max={100}
            value={brightness}
            disabled={isBrightnessLoading}
            onChange={(e) => handleBrightnessChange(Number(e.target.value))}
            className="w-full accent-brand-600"
          />
        </div>

        <Button variant="ghost" className="w-full" onClick={onBack}>
          Back
        </Button>
      </Card>
    </div>
  );
}
