import { TelemetryStreamFrame } from "../robot-link/robotLink.protocol";

/** The one method robot-link's gateway needs — kept as an interface so that module doesn't depend on telemetry-relay's concrete service/fan-out implementation. */
export interface ITelemetryRelay {
  handleRobotFrame(robotMongoId: string, frame: TelemetryStreamFrame): void;
}
