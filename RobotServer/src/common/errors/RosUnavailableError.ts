import { AppError } from "./AppError";

/**
 * The ROS side of a request (an action server, a service, a running
 * launch file) isn't reachable right now — e.g. Nav2 hasn't come up yet,
 * or the navigation container is still starting. Distinct from a 404:
 * the resource/route is valid, the robot just isn't ready for it yet.
 */
export class RosUnavailableError extends AppError {
  constructor(message = "The requested ROS component is not available right now") {
    super(message, 503);
  }
}
