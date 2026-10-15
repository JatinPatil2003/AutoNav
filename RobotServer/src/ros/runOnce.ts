import { spawn } from "node:child_process";

/** Runs a short-lived command to completion (e.g. `map_saver_cli`) — distinct from LaunchManager, which supervises long-running `ros2 launch` processes. */
export function runOnce(command: string, args: string[]): Promise<number> {
  return new Promise((resolve, reject) => {
    const proc = spawn(command, args);
    proc.on("exit", (code) => resolve(code ?? 1));
    proc.on("error", reject);
  });
}
