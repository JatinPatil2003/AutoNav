import { TeleopService } from "../../src/modules/teleop/teleop.service";
import { FakeRosTopicGateway } from "./fakes/FakeRosTopicGateway";

describe("TeleopService", () => {
  let topicGateway: FakeRosTopicGateway;
  let service: TeleopService;

  beforeEach(() => {
    topicGateway = new FakeRosTopicGateway();
    service = new TeleopService(topicGateway);
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
});
