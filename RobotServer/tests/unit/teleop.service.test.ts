import { NavStatus } from "../../src/modules/navigation/navigation.types";
import { TeleopService } from "../../src/modules/teleop/teleop.service";
import { FakeClock } from "./fakes/FakeClock";
import { FakeRosTopicGateway } from "./fakes/FakeRosTopicGateway";

const MAX_LINEAR = 1.0;
const MAX_ANGULAR = 1.5;

describe("TeleopService", () => {
  let topicGateway: FakeRosTopicGateway;
  let clock: FakeClock;
  let service: TeleopService;

  beforeEach(() => {
    topicGateway = new FakeRosTopicGateway();
    clock = new FakeClock();
    service = new TeleopService(topicGateway, MAX_LINEAR, MAX_ANGULAR, clock);
    topicGateway.published = []; // clear anything the constructor's createTimer setup may have recorded
  });

  it("publishes commanded velocity on cmd_vel", () => {
    service.setVelocity({ linear: 0.5, angular: -0.2 });
    service.publishTick(); // rising edge (inactive -> active): publishes an explicit zero once
    topicGateway.published = [];
    service.publishTick(); // steady state: publishes the actual commanded velocity

    expect(topicGateway.published).toContainEqual({
      type: "geometry_msgs/msg/Twist",
      topicName: "cmd_vel",
      message: {
        linear: { x: 0.5, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: -0.2 },
      },
    });
  });

  it("publishes an explicit zero once when velocity returns to zero", () => {
    service.setVelocity({ linear: 0.5, angular: 0 });
    service.publishTick();
    topicGateway.published = [];

    service.setVelocity({ linear: 0, angular: 0 });
    service.publishTick();

    expect(topicGateway.published).toEqual([
      {
        type: "geometry_msgs/msg/Twist",
        topicName: "cmd_vel",
        message: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
      },
    ]);
  });

  it("forwards emergency stop state immediately", () => {
    service.setEmergency(true);

    expect(topicGateway.published).toContainEqual({
      type: "std_msgs/msg/Bool",
      topicName: "/motor/emergency",
      message: { data: true },
    });
  });

  describe("per-command deadline (watchdog)", () => {
    it("keeps publishing the commanded velocity while within the TTL", () => {
      service.setVelocity({ linear: 0.3, angular: 0 }, 500);
      service.publishTick(); // rising edge
      clock.advanceBy(499);
      topicGateway.published = [];

      service.publishTick();

      expect(topicGateway.published).toEqual([
        {
          type: "geometry_msgs/msg/Twist",
          topicName: "cmd_vel",
          message: { linear: { x: 0.3, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
        },
      ]);
    });

    it("force-stops (edge-triggered zero) once the command's TTL expires without a refresh", () => {
      service.setVelocity({ linear: 0.3, angular: 0 }, 500);
      service.publishTick(); // rising edge
      topicGateway.published = [];
      clock.advanceBy(501);

      service.publishTick();

      expect(topicGateway.published).toEqual([
        {
          type: "geometry_msgs/msg/Twist",
          topicName: "cmd_vel",
          message: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
        },
      ]);
    });

    it("does not expire a long-running command before its own ttlMs (rotateInPlace's use case)", () => {
      service.setVelocity({ linear: 0, angular: 0.6 }, 10_000); // a long TTL, as rotateInPlace passes
      service.publishTick(); // rising edge
      topicGateway.published = [];
      clock.advanceBy(9_999);

      service.publishTick();

      expect(topicGateway.published).toEqual([
        {
          type: "geometry_msgs/msg/Twist",
          topicName: "cmd_vel",
          message: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0.6 } },
        },
      ]);
    });

    it("stop() forces an immediate stop regardless of any pending TTL", () => {
      service.setVelocity({ linear: 0.5, angular: 0 }, 10_000);
      service.publishTick();
      topicGateway.published = [];

      service.stop();
      service.publishTick();

      expect(topicGateway.published).toEqual([
        {
          type: "geometry_msgs/msg/Twist",
          topicName: "cmd_vel",
          message: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
        },
      ]);
    });
  });

  describe("numeric clamping", () => {
    it("clamps linear and angular velocity to the configured maximums", () => {
      service.setVelocity({ linear: 5, angular: -5 });
      service.publishTick(); // rising edge
      topicGateway.published = [];
      service.publishTick();

      expect(topicGateway.published).toEqual([
        {
          type: "geometry_msgs/msg/Twist",
          topicName: "cmd_vel",
          message: { linear: { x: MAX_LINEAR, y: 0, z: 0 }, angular: { x: 0, y: 0, z: -MAX_ANGULAR } },
        },
      ]);
    });
  });

  describe("Nav2-active refusal", () => {
    it("refuses (no-op) velocity commands while NavigationService reports an active goal", () => {
      service.bindNavigationStatus(() => NavStatus.ACTIVE);

      service.setVelocity({ linear: 0.5, angular: 0 });
      service.publishTick();

      expect(topicGateway.published).toEqual([]);
    });

    it("allows velocity commands once navigation is no longer ACTIVE", () => {
      let status = NavStatus.ACTIVE;
      service.bindNavigationStatus(() => status);

      service.setVelocity({ linear: 0.5, angular: 0 });
      service.publishTick();
      expect(topicGateway.published).toEqual([]);

      status = NavStatus.IDLE;
      service.setVelocity({ linear: 0.5, angular: 0 });
      service.publishTick(); // rising edge

      expect(topicGateway.published).toContainEqual({
        type: "geometry_msgs/msg/Twist",
        topicName: "cmd_vel",
        message: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
      });
    });

    it("stop() is never refused, even while navigation is ACTIVE", () => {
      service.bindNavigationStatus(() => NavStatus.ACTIVE);

      service.stop();

      expect(() => service.stop()).not.toThrow();
    });
  });
});
