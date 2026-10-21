import { useState } from "react";
import { skipToken } from "@reduxjs/toolkit/query/react";
import { Navigation2, Play, Square, Wifi, WifiOff } from "lucide-react";
import { useSelectedRobot } from "../../../app/robotSelection/useSelectedRobot";
import { getApiErrorMessage } from "../../../app/apiError";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { Input } from "../../../components/ui/Input";
import { PlaceholderPage } from "../../../components/ui/PlaceholderPage";
import {
  useCancelGoalMutation,
  useListGoalsQuery,
  useListMapsQuery,
  useSaveGoalMutation,
  useSelectMapMutation,
  useStartGoalMutation,
} from "../../navigation/navigationApi";
import { OccupancyGridCanvas } from "../../telemetry/OccupancyGridCanvas";
import { Pose2D } from "../../telemetry/telemetry.types";
import { useRobotTelemetry } from "../../telemetry/useRobotTelemetry";

export function NavigationPage() {
  const { selectedRobot } = useSelectedRobot();
  const robotId = selectedRobot?.id ?? null;
  const { pose, map, navStatus, robotOnline } = useRobotTelemetry(robotId);

  const [selectedMap, setSelectedMap] = useState<string>("");
  const [candidateGoal, setCandidateGoal] = useState<Pose2D | null>(null);
  const [goalName, setGoalName] = useState("");
  const [error, setError] = useState<string | null>(null);

  const { data: mapsData } = useListMapsQuery(robotId ? { robotId } : skipToken);
  const { data: goals } = useListGoalsQuery(robotId && selectedMap ? { robotId, mapName: selectedMap } : skipToken);

  const [selectMap, { isLoading: isSelectingMap }] = useSelectMapMutation();
  const [saveGoal, { isLoading: isSavingGoal }] = useSaveGoalMutation();
  const [startGoal, { isLoading: isStartingGoal }] = useStartGoalMutation();
  const [cancelGoal, { isLoading: isCancelling }] = useCancelGoalMutation();

  if (!selectedRobot) {
    return <PlaceholderPage title="Navigation" subtitle="Select a robot first" />;
  }
  const confirmedRobotId = selectedRobot.id;

  const runAction = async (action: () => Promise<unknown>): Promise<void> => {
    setError(null);
    try {
      await action();
    } catch (err) {
      setError(getApiErrorMessage(err as Parameters<typeof getApiErrorMessage>[0]));
    }
  };

  const handleSelectMap = (mapName: string) =>
    runAction(async () => {
      setSelectedMap(mapName);
      if (mapName) await selectMap({ robotId: confirmedRobotId, mapName }).unwrap();
    });

  const handleSaveGoal = () =>
    runAction(async () => {
      if (!candidateGoal || !goalName.trim() || !selectedMap) return;
      await saveGoal({ robotId: confirmedRobotId, mapName: selectedMap, name: goalName.trim(), ...candidateGoal }).unwrap();
      setCandidateGoal(null);
      setGoalName("");
    });

  const handleStartGoal = (name: string) =>
    runAction(() => startGoal({ robotId: confirmedRobotId, mapName: selectedMap, name }).unwrap());

  const handleCancelGoal = () => runAction(() => cancelGoal({ robotId: confirmedRobotId }).unwrap());

  const allGoalNames = goals ? [...goals.localization, ...goals.charging, ...goals.standby, ...goals.goals] : [];

  return (
    <div className="flex flex-col gap-4 p-3">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <h1 className="flex items-center gap-2 text-xl font-semibold text-slate-900 dark:text-slate-100">
          <Navigation2 className="h-5 w-5" />
          Navigation — {selectedRobot.displayName ?? selectedRobot.name}
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
        <Card className="flex flex-col gap-2 p-3">
          <p className="text-xs text-slate-500 dark:text-slate-400">Click the map to drop a candidate goal, then name and save it below.</p>
          <OccupancyGridCanvas map={map} pose={pose} poseConfident={pose?.confident} goalPose={candidateGoal} onSelectPose={setCandidateGoal} className="w-full" />
        </Card>

        <div className="flex flex-col gap-4">
          <Card>
            <h2 className="mb-3 text-sm font-medium text-slate-500 dark:text-slate-400">Map</h2>
            <select
              value={selectedMap}
              onChange={(e) => handleSelectMap(e.target.value)}
              disabled={isSelectingMap}
              className="w-full rounded-xl border border-slate-300/80 bg-white/80 px-3 py-2 text-sm text-slate-900 shadow-sm dark:border-slate-600 dark:bg-slate-900/70 dark:text-slate-100"
            >
              <option value="">Select a saved map…</option>
              {(mapsData?.maps ?? []).map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
            </select>
          </Card>

          <Card>
            <h2 className="mb-3 text-sm font-medium text-slate-500 dark:text-slate-400">Save candidate goal</h2>
            <div className="flex flex-col gap-3">
              <Input
                label="Name"
                placeholder="e.g. loading-bay"
                value={goalName}
                onChange={(e) => setGoalName(e.target.value)}
                disabled={!candidateGoal || !selectedMap}
              />
              <Button onClick={handleSaveGoal} isLoading={isSavingGoal} disabled={!candidateGoal || !goalName.trim() || !selectedMap}>
                Save goal at clicked point
              </Button>
              {!selectedMap && <p className="text-xs text-amber-600 dark:text-amber-400">Select a map first.</p>}
            </div>
          </Card>

          <Card>
            <h2 className="mb-3 text-sm font-medium text-slate-500 dark:text-slate-400">Goals on this map</h2>
            <div className="flex flex-col gap-2">
              {allGoalNames.length === 0 && <p className="text-sm text-slate-500 dark:text-slate-400">No saved goals yet.</p>}
              {allGoalNames.map((name) => (
                <div key={name} className="flex items-center justify-between gap-2">
                  <span className="truncate text-sm text-slate-700 dark:text-slate-300">{name}</span>
                  <Button
                    variant="secondary"
                    onClick={() => handleStartGoal(name)}
                    isLoading={isStartingGoal}
                    disabled={navStatus?.status === "ACTIVE"}
                  >
                    <Play className="h-3.5 w-3.5" /> Go
                  </Button>
                </div>
              ))}
            </div>
          </Card>

          <Card>
            <h2 className="mb-3 text-sm font-medium text-slate-500 dark:text-slate-400">Active goal</h2>
            {navStatus?.status === "ACTIVE" ? (
              <div className="flex flex-col gap-2">
                <p className="text-sm text-slate-700 dark:text-slate-300">
                  Navigating to <span className="font-medium">{navStatus.activeGoalName ?? "unknown"}</span>
                  {navStatus.feedbackDistanceRemaining != null && ` — ${navStatus.feedbackDistanceRemaining.toFixed(2)}m remaining`}
                </p>
                <Button variant="danger" onClick={handleCancelGoal} isLoading={isCancelling}>
                  <Square className="h-4 w-4" /> Cancel
                </Button>
              </div>
            ) : (
              <p className="text-sm text-slate-500 dark:text-slate-400">{navStatus?.status ?? "IDLE"}</p>
            )}
          </Card>

          {error && <p className="text-sm text-red-600 dark:text-red-400">{error}</p>}
        </div>
      </div>
    </div>
  );
}
