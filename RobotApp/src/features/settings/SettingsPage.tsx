import { useState } from "react";
import { ArrowLeft, Lightbulb, SlidersHorizontal, Wifi } from "lucide-react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { AppShell } from "../../components/ui/AppShell";
import { getApiErrorMessage } from "../../api/apiError";
import { LED_PATTERN_IDLE, LED_PATTERN_OFF, firmwareClient } from "../../api/firmwareClient";

interface SettingsPageProps {
  onBack: () => void;
  onOpenWifi: () => void;
}

/**
 * Display controls for this robot's LED strip, via RobotServer's
 * /api/firmware/led/* (the ESP32 bridge over ROS2 — see firmware.service.ts).
 * There's no "get current state" endpoint on the firmware bridge, so this
 * page tracks light/brightness optimistically rather than reading them back.
 */
export function SettingsPage({ onBack, onOpenWifi }: SettingsPageProps) {
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
    <AppShell>
      <Card>
        <h1 className="mb-1 text-2xl font-bold tracking-tight text-slate-900 dark:text-slate-50">
          Settings
        </h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">LED controls for this robot.</p>

        {error && (
          <p className="mb-4 rounded-lg bg-red-50 px-3 py-2 text-sm text-red-600 dark:bg-red-900/20 dark:text-red-400">
            {error}
          </p>
        )}

        <div className="mb-4 flex items-center justify-between rounded-xl border border-slate-200/70 bg-white/60 px-4 py-3 dark:border-slate-700/70 dark:bg-slate-800/40">
          <span className="flex items-center gap-2 text-sm font-medium text-slate-900 dark:text-slate-100">
            <Lightbulb className={`h-4 w-4 ${lightOn ? "text-amber-500" : "text-slate-400"}`} />
            Light
          </span>
          <Button variant={lightOn ? "primary" : "secondary"} isLoading={isLightLoading} onClick={handleToggleLight}>
            {lightOn ? "On" : "Off"}
          </Button>
        </div>

        <div className="mb-6 rounded-xl border border-slate-200/70 bg-white/60 px-4 py-3 dark:border-slate-700/70 dark:bg-slate-800/40">
          <div className="mb-2 flex items-center justify-between">
            <span className="flex items-center gap-2 text-sm font-medium text-slate-900 dark:text-slate-100">
              <SlidersHorizontal className="h-4 w-4 text-slate-400" />
              Brightness
            </span>
            <span className="text-sm font-semibold text-brand-600 dark:text-brand-400">{brightness}%</span>
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

        <Button variant="secondary" className="mb-3 w-full" onClick={onOpenWifi}>
          <Wifi className="h-4 w-4" /> Wifi settings
        </Button>

        <Button variant="ghost" className="w-full" onClick={onBack}>
          <ArrowLeft className="h-4 w-4" /> Back
        </Button>
      </Card>
    </AppShell>
  );
}
