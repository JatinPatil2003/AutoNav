import { useEffect, useRef } from "react";
import { MapSnapshotFrame, Pose2D } from "./telemetry.types";

interface OccupancyGridCanvasProps {
  map: MapSnapshotFrame | null;
  /** The robot's own live pose, if known. */
  pose?: Pose2D | null;
  poseConfident?: boolean;
  /** A candidate/active navigation goal pose, drawn in a different color from the robot. */
  goalPose?: Pose2D | null;
  /** Fires with a world-frame pose (theta always 0 — orientation isn't settable by a single click) when the canvas is clicked and a map is loaded. */
  onSelectPose?: (pose: Pose2D) => void;
  className?: string;
}

const CANVAS_MAX_WIDTH_PX = 900;
const ROBOT_MARKER_SIZE_PX = 16;
const GOAL_MARKER_SIZE_PX = 14;

function occupancyColor(value: number): string {
  if (value < 0) return "#9ca3af"; // unknown — slate-400
  const shade = 255 - Math.round((Math.min(value, 100) / 100) * 255);
  return `rgb(${shade}, ${shade}, ${shade})`;
}

/** World (x,y) -> canvas pixel. OccupancyGrid row 0 sits at the origin (world y-minimum), so row/y is flipped onto canvas pixels (canvas y grows downward, world y grows "up"). */
function worldToCanvas(
  worldX: number,
  worldY: number,
  info: MapSnapshotFrame["info"],
  cellWidth: number,
  cellHeight: number,
): { x: number; y: number } {
  const col = (worldX - info.origin.position.x) / info.resolution;
  const row = (worldY - info.origin.position.y) / info.resolution;
  return { x: col * cellWidth, y: (info.height - row) * cellHeight };
}

function canvasToWorld(
  canvasX: number,
  canvasY: number,
  info: MapSnapshotFrame["info"],
  cellWidth: number,
  cellHeight: number,
): Pose2D {
  const col = canvasX / cellWidth;
  const row = info.height - canvasY / cellHeight;
  return {
    x: info.origin.position.x + col * info.resolution,
    y: info.origin.position.y + row * info.resolution,
    theta: 0,
  };
}

function drawMarker(ctx: CanvasRenderingContext2D, x: number, y: number, theta: number, size: number, color: string): void {
  // Canvas y is flipped relative to world y, so a world heading theta
  // (CCW-positive from +x) points in screen direction (cos theta, -sin theta).
  const dx = Math.cos(theta);
  const dy = -Math.sin(theta);
  const angle = Math.atan2(dy, dx);

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(angle);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(size / 2, 0);
  ctx.lineTo(-size / 2, size / 2.5);
  ctx.lineTo(-size / 2, -size / 2.5);
  ctx.closePath();
  ctx.fill();
  ctx.strokeStyle = "rgba(0,0,0,0.3)";
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.restore();
}

export function OccupancyGridCanvas({ map, pose, poseConfident, goalPose, onSelectPose, className = "" }: OccupancyGridCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !map) return;

    const { info, data } = map;
    const width = Math.min(CANVAS_MAX_WIDTH_PX, info.width * 4);
    const height = Math.round((width * info.height) / info.width);
    canvas.width = width;
    canvas.height = height;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const cellWidth = width / info.width;
    const cellHeight = height / info.height;

    ctx.clearRect(0, 0, width, height);
    for (let row = 0; row < info.height; row += 1) {
      for (let col = 0; col < info.width; col += 1) {
        const value = data[row * info.width + col];
        if (value === 0) continue; // free space — leave the canvas's own background showing through
        ctx.fillStyle = occupancyColor(value);
        ctx.fillRect(col * cellWidth, (info.height - 1 - row) * cellHeight, cellWidth, cellHeight);
      }
    }

    if (goalPose) {
      const { x, y } = worldToCanvas(goalPose.x, goalPose.y, info, cellWidth, cellHeight);
      drawMarker(ctx, x, y, goalPose.theta, GOAL_MARKER_SIZE_PX, "#f59e0b");
    }

    if (pose) {
      const { x, y } = worldToCanvas(pose.x, pose.y, info, cellWidth, cellHeight);
      drawMarker(ctx, x, y, pose.theta, ROBOT_MARKER_SIZE_PX, poseConfident === false ? "#94a3b8" : "#2563eb");
    }
  }, [map, pose, poseConfident, goalPose]);

  const handleClick = (event: React.MouseEvent<HTMLCanvasElement>): void => {
    if (!onSelectPose || !map) return;
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const canvasX = (event.clientX - rect.left) * scaleX;
    const canvasY = (event.clientY - rect.top) * scaleY;

    const cellWidth = canvas.width / map.info.width;
    const cellHeight = canvas.height / map.info.height;
    onSelectPose(canvasToWorld(canvasX, canvasY, map.info, cellWidth, cellHeight));
  };

  if (!map) {
    return (
      <div className={`flex items-center justify-center rounded-xl bg-slate-100 text-sm text-slate-500 dark:bg-slate-800/60 dark:text-slate-400 ${className}`}>
        No map data yet
      </div>
    );
  }

  return (
    <canvas
      ref={canvasRef}
      onClick={handleClick}
      className={`max-w-full rounded-xl border border-slate-200/70 bg-white dark:border-slate-700/70 ${onSelectPose ? "cursor-crosshair" : ""} ${className}`}
    />
  );
}
