import { WebSocket } from "ws";
import { TelemetryRelayService } from "../../src/modules/telemetry-relay/telemetryRelay.service";
import { PoseUpdateFrame } from "../../src/modules/robot-link/robotLink.protocol";
import { FakeRobotConnectionRegistry } from "./fakes/FakeRobotConnectionRegistry";
import { FakeRobotRepository } from "./fakes/FakeRobotRepository";

/** Minimal stand-in for a dashboard viewer's socket — just enough surface for TelemetryRelayService. */
class FakeViewerSocket {
  public readyState: number = WebSocket.OPEN;
  public received: unknown[] = [];
  private handlers = new Map<string, Array<(...args: unknown[]) => void>>();

  on(event: string, handler: (...args: unknown[]) => void): this {
    const list = this.handlers.get(event) ?? [];
    list.push(handler);
    this.handlers.set(event, list);
    return this;
  }

  send(data: string): void {
    this.received.push(JSON.parse(data));
  }

  close(): void {
    this.readyState = WebSocket.CLOSED;
    this.emit("close");
  }

  emit(event: string, ...args: unknown[]): void {
    for (const handler of this.handlers.get(event) ?? []) handler(...args);
  }
}

describe("TelemetryRelayService", () => {
  let registry: FakeRobotConnectionRegistry;
  let robotRepository: FakeRobotRepository;
  let service: TelemetryRelayService;
  let robotMongoId: string;
  const robotSlug = "bot-1";

  const poseFrame: PoseUpdateFrame = { type: "pose.update", x: 1, y: 2, theta: 0, source: "amcl", confident: true };

  beforeEach(async () => {
    registry = new FakeRobotConnectionRegistry();
    robotRepository = new FakeRobotRepository();
    service = new TelemetryRelayService(registry, robotRepository);

    const robot = await robotRepository.create({
      robotId: robotSlug,
      name: "Bot One",
      vpnHostname: "bot-1.vpn.internal",
      displayName: "Bot One",
    });
    robotMongoId = robot._id.toString();
    registry.setOnline(robotSlug, true);
  });

  it("sends an initial robot_link_state frame reflecting the current online status", () => {
    const viewer = new FakeViewerSocket();
    service.subscribe(robotMongoId, robotSlug, viewer as unknown as WebSocket);

    expect(viewer.received).toEqual([{ type: "robot_link_state", online: true }]);
  });

  it("fans a robot frame out to every subscribed viewer, not just the first", () => {
    const viewerA = new FakeViewerSocket();
    const viewerB = new FakeViewerSocket();
    service.subscribe(robotMongoId, robotSlug, viewerA as unknown as WebSocket);
    service.subscribe(robotMongoId, robotSlug, viewerB as unknown as WebSocket);

    service.handleRobotFrame(robotMongoId, poseFrame);

    expect(viewerA.received).toContainEqual(poseFrame);
    expect(viewerB.received).toContainEqual(poseFrame);
  });

  it("does not preempt an earlier viewer when a second one subscribes — both keep receiving frames", () => {
    const viewerA = new FakeViewerSocket();
    const viewerB = new FakeViewerSocket();
    service.subscribe(robotMongoId, robotSlug, viewerA as unknown as WebSocket);
    service.handleRobotFrame(robotMongoId, poseFrame);
    service.subscribe(robotMongoId, robotSlug, viewerB as unknown as WebSocket);

    service.handleRobotFrame(robotMongoId, { ...poseFrame, x: 5 });

    expect(viewerA.readyState).toBe(WebSocket.OPEN);
    expect(viewerA.received).toHaveLength(3); // initial state + first pose + second pose
    expect(viewerB.received).toHaveLength(2); // initial state + second pose only
  });

  it("stops sending to a viewer once its socket closes", () => {
    const viewer = new FakeViewerSocket();
    service.subscribe(robotMongoId, robotSlug, viewer as unknown as WebSocket);
    viewer.close();

    service.handleRobotFrame(robotMongoId, poseFrame);

    expect(viewer.received).not.toContainEqual(poseFrame);
  });

  it("notifies every subscribed viewer when the robot-link disconnects", async () => {
    const viewer = new FakeViewerSocket();
    service.subscribe(robotMongoId, robotSlug, viewer as unknown as WebSocket);

    registry.simulateDisconnect(robotSlug);
    // The disconnect handler chains an async repository lookup through a
    // .catch() before notifying viewers — several microtask hops, not one.
    // setImmediate defers to the next macrotask, by which point the whole
    // microtask queue (however many .then/.catch links deep) has drained.
    await new Promise((resolve) => setImmediate(resolve));

    expect(viewer.received).toContainEqual({ type: "robot_link_state", online: false });
  });

  it("does nothing when a frame arrives for a robot with no subscribed viewers", () => {
    expect(() => service.handleRobotFrame(robotMongoId, poseFrame)).not.toThrow();
  });
});
