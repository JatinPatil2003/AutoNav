import { TopicFrequencyMonitor } from "../../src/ros/topicFrequencyMonitor";
import { FakeRosTopicGateway } from "./fakes/FakeRosTopicGateway";
import { FakeClock } from "./fakes/FakeClock";

describe("TopicFrequencyMonitor", () => {
  let topicGateway: FakeRosTopicGateway;
  let clock: FakeClock;
  let monitor: TopicFrequencyMonitor;

  beforeEach(() => {
    topicGateway = new FakeRosTopicGateway();
    clock = new FakeClock();
    monitor = new TopicFrequencyMonitor(topicGateway, clock);
  });

  it("reports null Hz for a topic that's never received a message", () => {
    monitor.track("sensor_msgs/msg/LaserScan", "/scan");

    expect(monitor.getHz("/scan")).toBeNull();
    expect(monitor.getLastMessageAgeMs("/scan")).toBeNull();
  });

  it("reports null Hz after exactly one message — needs a gap between two to measure anything", () => {
    monitor.track("sensor_msgs/msg/LaserScan", "/scan");
    topicGateway.fireMessage("/scan", {});

    expect(monitor.getHz("/scan")).toBeNull();
    expect(monitor.getLastMessageAgeMs("/scan")).toBe(0);
  });

  it("computes Hz from the average inter-message gap", () => {
    monitor.track("sensor_msgs/msg/LaserScan", "/scan");

    topicGateway.fireMessage("/scan", {});
    clock.advanceBy(100); // 10Hz-equivalent gap
    topicGateway.fireMessage("/scan", {});
    clock.advanceBy(100);
    topicGateway.fireMessage("/scan", {});

    expect(monitor.getHz("/scan")).toBeCloseTo(10);
  });

  it("tracks multiple topics independently", () => {
    monitor.track("sensor_msgs/msg/LaserScan", "/scan");
    monitor.track("sensor_msgs/msg/Imu", "autonav_imu/imu");

    topicGateway.fireMessage("/scan", {});
    clock.advanceBy(50); // 20Hz
    topicGateway.fireMessage("/scan", {});

    clock.advanceBy(200); // 5Hz from imu's first message onward
    topicGateway.fireMessage("autonav_imu/imu", {});
    clock.advanceBy(200);
    topicGateway.fireMessage("autonav_imu/imu", {});

    expect(monitor.getHz("/scan")).toBeCloseTo(20);
    expect(monitor.getHz("autonav_imu/imu")).toBeCloseTo(5);
  });

  it("reports the age of the last message as time passes without a new one", () => {
    monitor.track("sensor_msgs/msg/LaserScan", "/scan");
    topicGateway.fireMessage("/scan", {});

    clock.advanceBy(3_000);

    expect(monitor.getLastMessageAgeMs("/scan")).toBe(3_000);
  });

  it("is idempotent — tracking the same topic twice doesn't double-count messages", () => {
    monitor.track("sensor_msgs/msg/LaserScan", "/scan");
    monitor.track("sensor_msgs/msg/LaserScan", "/scan");

    topicGateway.fireMessage("/scan", {});
    clock.advanceBy(100);
    topicGateway.fireMessage("/scan", {});

    expect(monitor.getHz("/scan")).toBeCloseTo(10);
  });
});
