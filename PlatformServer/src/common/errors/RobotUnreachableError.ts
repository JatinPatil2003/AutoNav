import { AppError } from "./AppError";

/** The robot's own RobotServer REST API didn't answer (offline, VPN down, or rejected the request) — mirrors RobotServer's RosUnavailableError for the proxy's half of this relationship. */
export class RobotUnreachableError extends AppError {
  constructor(message = "Robot did not respond") {
    super(message, 502);
  }
}
