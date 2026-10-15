import { ChildProcess, spawn } from "node:child_process";
import { ConflictError } from "../common/errors";
import { logger } from "../common/utils/logger.util";

export interface LaunchStatus {
  running: boolean;
  name?: string;
}

/**
 * Single source of truth for `ros2 launch` subprocesses. The old WebServer
 * had three separate ad-hoc Popen/killpg blocks (bringup, navigation,
 * mapping), each with its own global `process` variable and no awareness
 * of each other — nothing stopped bringup, nav, and SLAM all launching at
 * once. This enforces "only one launch at a time" and is shared by every
 * module that needs to start one.
 */
export class LaunchManager {
  private current: { name: string; proc: ChildProcess } | undefined;

  start(name: string, command: string, args: string[]): void {
    if (this.current) {
      if (this.current.name === name) {
        return; // already running this exact launch — idempotent
      }
      throw new ConflictError(`Cannot start "${name}": "${this.current.name}" is already running`);
    }

    const proc = spawn(command, args, { detached: true, stdio: "ignore" });
    proc.on("exit", (code) => {
      logger.info({ name, code }, "ros2 launch process exited");
      if (this.current?.proc === proc) {
        this.current = undefined;
      }
    });
    proc.on("error", (err) => {
      logger.error({ name, err }, "ros2 launch process failed to start");
      if (this.current?.proc === proc) {
        this.current = undefined;
      }
    });

    this.current = { name, proc };
    logger.info({ name, command, args }, "ros2 launch process started");
  }

  stop(): void {
    if (!this.current) {
      return;
    }
    const { proc, name } = this.current;
    if (proc.pid) {
      process.kill(-proc.pid, "SIGTERM");
    }
    this.current = undefined;
    logger.info({ name }, "ros2 launch process stopped");
  }

  status(): LaunchStatus {
    return this.current ? { running: true, name: this.current.name } : { running: false };
  }
}
