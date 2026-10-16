import { IPlatformLink } from "../../clients/platformLink.interface";
import { logger } from "../../common/utils/logger.util";
import { DeviceIdentityService } from "../../ros/deviceIdentity";
import { TopicGateway } from "../../ros/rosTopicGateway";
import { NavigationService } from "../navigation/navigation.service";
import { NavStatus } from "../navigation/navigation.types";
import { TeleopService } from "./teleop.service";
import { HealthResultFrame, inboundLinkFrameSchema } from "./teleopLink.protocol";

export interface TeleopLinkDeps {
  link: IPlatformLink;
  teleopService: TeleopService;
  navigationService: NavigationService;
  deviceIdentity: DeviceIdentityService;
  topicGateway: TopicGateway;
}

/**
 * The replacement for the old (deleted, and staying deleted) `/ws/teleop`
 * inbound WebSocket server. Remote clients no longer reach RobotServer
 * directly — this instead subscribes to frames arriving over the Part 2
 * `platformLink` connection and dispatches them, so the robot's only
 * exposure is the one outbound-initiated link in platformLink.client.ts.
 */
export function registerTeleopLinkHandler(deps: TeleopLinkDeps): void {
  const { link, teleopService, navigationService, deviceIdentity, topicGateway } = deps;

  link.onFrame((raw) => {
    const parsed = inboundLinkFrameSchema.safeParse(raw);
    if (!parsed.success) {
      logger.debug({ raw, issues: parsed.error.issues }, "teleopLink: dropping invalid frame");
      return;
    }

    const frame = parsed.data;
    switch (frame.type) {
      case "teleop.velocity":
        logger.debug({ sessionId: frame.sessionId, data: frame.data }, "teleopLink: velocity");
        teleopService.setVelocity(frame.data);
        break;

      case "teleop.estop":
        logger.info({ sessionId: frame.sessionId, engaged: frame.engaged }, "teleopLink: estop");
        teleopService.setEmergency(frame.engaged);
        break;

      case "teleop.release":
        logger.info({ sessionId: frame.sessionId }, "teleopLink: release");
        teleopService.stop();
        break;

      case "health.query": {
        const reply: HealthResultFrame = {
          type: "health.result",
          id: frame.id,
          rclOk: topicGateway.isNodeAlive() && deviceIdentity.getRobotId() !== null,
          navigationActive: navigationService.getStatus() === NavStatus.ACTIVE,
          ...teleopService.getLimits(),
        };
        link.send(reply);
        break;
      }
    }
  });
}
