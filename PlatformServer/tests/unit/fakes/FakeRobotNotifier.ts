import { IRobotNotifier } from "../../../src/modules/robot-assignments/robotNotifier.interface";

export class FakeRobotNotifier implements IRobotNotifier {
  public notified: string[] = [];

  async notifyRosterInvalidate(robotId: string): Promise<void> {
    this.notified.push(robotId);
  }
}
