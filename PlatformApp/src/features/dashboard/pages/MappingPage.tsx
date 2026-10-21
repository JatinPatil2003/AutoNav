import { useState } from "react";
import { Map as MapIcon, Play, Save, Square, Wifi, WifiOff } from "lucide-react";
import { useSelectedRobot } from "../../../app/robotSelection/useSelectedRobot";
import { getApiErrorMessage } from "../../../app/apiError";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { Input } from "../../../components/ui/Input";
import { PlaceholderPage } from "../../../components/ui/PlaceholderPage";
import {
  useCaptureWaypointMutation,
  useCommitWaypointsMutation,
  useSaveMapMutation,
  useStartMappingMutation,
  useStopMappingMutation,
} from "../../mapping/mappingApi";
import { OccupancyGridCanvas } from "../../telemetry/OccupancyGridCanvas";
import { SlamBackend } from "../../telemetry/telemetry.types";
import { useRobotTelemetry } from "../../telemetry/useRobotTelemetry";

export function MappingPage() {
  const { selectedRobot } = useSelectedRobot();
  const robotId = selectedRobot?.id ?? null;
  const { pose, map, robotStatus, robotOnline } = useRobotTelemetry(robotId);

  const [backend, setBackend] = useState<SlamBackend>("cartographer");
  const [waypointName, setWaypointName] = useState("");
  const [mapName, setMapName] = useState("");
  const [error, setError] = useState<string | null>(null);

  const [startMapping, { isLoading: isStarting }] = useStartMappingMutation();
  const [stopMapping, { isLoading: isStopping }] = useStopMappingMutation();
  const [captureWaypoint, { isLoading: isCapturing }] = useCaptureWaypointMutation();
  const [saveMap, { isLoading: isSaving }] = useSaveMapMutation();
  const [commitWaypoints] = useCommitWaypointsMutation();

  if (!selectedRobot) {
    return <PlaceholderPage title="Mapping" subtitle="Select a robot first" />;
  }

  const mappingActive = robotStatus?.mappingActive ?? false;
  // selectedRobot is narrowed non-null past the early return above, but
  // robotId (derived from it earlier, before hooks could be called
  // conditionally) isn't automatically re-narrowed by TS — read it fresh
  // here so every handler below gets a plain `string`.
  const confirmedRobotId = selectedRobot.id;

  const runAction = async (action: () => Promise<unknown>): Promise<void> => {
    setError(null);
    try {
      await action();
    } catch (err) {
      setError(getApiErrorMessage(err as Parameters<typeof getApiErrorMessage>[0]));
    }
  };

  const handleStart = () => runAction(() => startMapping({ robotId: confirmedRobotId, backend }).unwrap());
  const handleStop = () => runAction(() => stopMapping({ robotId: confirmedRobotId }).unwrap());

  const handleCaptureWaypoint = () =>
    runAction(async () => {
      if (!waypointName.trim()) return;
      await captureWaypoint({ robotId: confirmedRobotId, name: waypointName.trim() }).unwrap();
      setWaypointName("");
    });

  const handleSaveMap = () =>
    runAction(async () => {
      if (!mapName.trim()) return;
      const trimmed = mapName.trim();
      await saveMap({ robotId: confirmedRobotId, mapName: trimmed }).unwrap();
      await commitWaypoints({ robotId: confirmedRobotId, mapName: trimmed }).unwrap();
      await stopMapping({ robotId: confirmedRobotId }).unwrap();
      setMapName("");
    });

  return (
    <div className="flex flex-col gap-4 p-3">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <h1 className="flex items-center gap-2 text-xl font-semibold text-slate-900 dark:text-slate-100">
          <MapIcon className="h-5 w-5" />
          Mapping — {selectedRobot.displayName ?? selectedRobot.name}
        </h1>
        <span
          className={`flex items-center gap-1 rounded-full px-2 py-1 text-xs font-medium ${
            robotOnline
              ? "bg-emerald-50 text-emerald-600 dark:bg-emerald-500/10 dark:text-emerald-400"
              : "bg-slate-100 text-slate-500 dark:bg-slate-700/50 dark:text-slate-400"
          }`}
        >
          {robotOnline ? <Wifi className="h-3.5 w-3.5" /> : <WifiOff className="h-3.5 w-3.5" />}
          {robotOnline === null ? "connecting…" : robotOnline ? "online" : "offline"}
        </span>
      </div>

      <div className="grid grid-cols-1 gap-4 lg:grid-cols-[2fr_1fr]">
        <Card className="flex items-center justify-center p-3">
          <OccupancyGridCanvas map={map} pose={pose} poseConfident={pose?.confident} className="w-full" />
        </Card>

        <div className="flex flex-col gap-4">
          <Card>
            <h2 className="mb-3 text-sm font-medium text-slate-500 dark:text-slate-400">SLAM session</h2>
            <div className="flex flex-col gap-3">
              <label className="flex flex-col gap-1 text-sm font-medium text-slate-700 dark:text-slate-300">
                Backend
                <select
                  value={backend}
                  disabled={mappingActive}
                  onChange={(e) => setBackend(e.target.value as SlamBackend)}
                  className="rounded-xl border border-slate-300/80 bg-white/80 px-3 py-2 text-sm text-slate-900 shadow-sm dark:border-slate-600 dark:bg-slate-900/70 dark:text-slate-100"
                >
                  <option value="cartographer">Cartographer (default)</option>
                  <option value="slam_toolbox">slam_toolbox</option>
                </select>
              </label>
              {!mappingActive ? (
                <Button onClick={handleStart} isLoading={isStarting}>
                  <Play className="h-4 w-4" /> Start mapping
                </Button>
              ) : (
                <Button variant="danger" onClick={handleStop} isLoading={isStopping}>
                  <Square className="h-4 w-4" /> Stop mapping
                </Button>
              )}
              {robotStatus?.activeBackend && (
                <p className="text-xs text-slate-500 dark:text-slate-400">Active backend: {robotStatus.activeBackend}</p>
              )}
            </div>
          </Card>

          <Card>
            <h2 className="mb-3 text-sm font-medium text-slate-500 dark:text-slate-400">Capture waypoint</h2>
            <div className="flex flex-col gap-3">
              <Input
                label="Name"
                placeholder="e.g. doorway"
                value={waypointName}
                onChange={(e) => setWaypointName(e.target.value)}
                disabled={!mappingActive}
              />
              <Button onClick={handleCaptureWaypoint} isLoading={isCapturing} disabled={!mappingActive || !waypointName.trim()}>
                Capture at current pose
              </Button>
              {!pose && mappingActive && (
                <p className="text-xs text-amber-600 dark:text-amber-400">No live pose yet — wait for tf_pose_bridge to publish.</p>
              )}
            </div>
          </Card>

          <Card>
            <h2 className="mb-3 text-sm font-medium text-slate-500 dark:text-slate-400">Finish &amp; save map</h2>
            <div className="flex flex-col gap-3">
              <Input
                label="Map name"
                placeholder="e.g. warehouse"
                value={mapName}
                onChange={(e) => setMapName(e.target.value)}
                disabled={!mappingActive}
              />
              <Button onClick={handleSaveMap} isLoading={isSaving} disabled={!mappingActive || !mapName.trim()}>
                <Save className="h-4 w-4" /> Save, commit waypoints &amp; stop
              </Button>
            </div>
          </Card>

          {error && <p className="text-sm text-red-600 dark:text-red-400">{error}</p>}
        </div>
      </div>
    </div>
  );
}
