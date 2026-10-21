import { RobotUnreachableError } from "../../../src/common/errors";
import { IRobotDocument } from "../../../src/modules/robots/robot.model";
import { IRobotApiClient } from "../../../src/modules/robot-proxy/robotApiClient.interface";

export interface RecordedCall {
  robotId: string;
  method: "GET" | "POST";
  path: string;
  body?: unknown;
}

export class FakeRobotApiClient implements IRobotApiClient {
  calls: RecordedCall[] = [];
  /** Set to make the next (or every) call throw, simulating an unreachable robot. */
  shouldFail = false;
  /** Returned from every call unless overridden per-test. */
  response: unknown = {};

  async request<TResponse>(robot: IRobotDocument, method: "GET" | "POST", path: string, body?: unknown): Promise<TResponse> {
    this.calls.push({ robotId: robot.robotId, method, path, body });
    if (this.shouldFail) {
      throw new RobotUnreachableError("simulated robot-unreachable failure");
    }
    return this.response as TResponse;
  }
}
