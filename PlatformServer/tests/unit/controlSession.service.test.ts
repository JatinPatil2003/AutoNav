import { WebSocket } from "ws";
import { ControlSessionService } from "../../src/modules/control/controlSession.service";
import { RobotAssignmentService } from "../../src/modules/robot-assignments/robotAssignment.service";
import { FakeRobotConnectionRegistry } from "./fakes/FakeRobotConnectionRegistry";
import { FakeRobotAssignmentRepository } from "./fakes/FakeRobotAssignmentRepository";
import { FakeRobotRepository } from "./fakes/FakeRobotRepository";
import { FakeRobotNotifier } from "./fakes/FakeRobotNotifier";

/** Minimal stand-in for the client-side `ws` socket — just enough surface for ControlSessionService. */
class FakeClientSocket {
  public readyState: number = WebSocket.OPEN;
  public sent: unknown[] = [];
  public closed = false;
  private handlers = new Map<string, Array<(...args: unknown[]) => void>>();

  on(event: string, handler: (...args: unknown[]) => void): this {
    const list = this.handlers.get(event) ?? [];
    list.push(handler);
    this.handlers.set(event, list);
    return this;
  }

  send(data: string): void {
    this.sent.push(JSON.parse(data));
  }

  close(): void {
    this.closed = true;
    this.readyState = WebSocket.CLOSED;
    this.emit("close");
  }

  emit(event: string, ...args: unknown[]): void {
    for (const handler of this.handlers.get(event) ?? []) handler(...args);
  }

  receiveFromClient(frame: unknown): void {
    this.emit("message", Buffer.from(JSON.stringify(frame)));
  }
}

describe("ControlSessionService", () => {
  let registry: FakeRobotConnectionRegistry;
  let assignmentService: RobotAssignmentService;
  let assignmentRepository: FakeRobotAssignmentRepository;
  let robotRepository: FakeRobotRepository;
  let service: ControlSessionService;
  let robotMongoId: string;
  const robotSlug = "bot-1";
  const userId = "507f1f77bcf86cd799439011";

  beforeEach(async () => {
    jest.useFakeTimers();
    robotRepository = new FakeRobotRepository();
    const robot = await robotRepository.create({
      robotId: robotSlug,
      name: "Bot 1",
      vpnHostname: "bot-1.robot",
    });
    robotMongoId = robot._id.toString();

    assignmentRepository = new FakeRobotAssignmentRepository(robotRepository);
    assignmentService = new RobotAssignmentService(assignmentRepository, robotRepository, new FakeRobotNotifier());
    await assignmentRepository.upsertActive(userId, robotMongoId, "507f1f77bcf86cd799439099", {
      platformAccess: true,
    });

    registry = new FakeRobotConnectionRegistry();
    registry.setOnline(robotSlug, true);
    service = new ControlSessionService(registry, assignmentService);
  });

  afterEach(() => {
    jest.useRealTimers();
  });

  function startSession(ws: FakeClientSocket): void {
    service.startSession({
      userId,
      robotMongoId,
      robotSlug,
      ws: ws as unknown as WebSocket,
    });
  }

  it("sends a ready frame on session start", () => {
    const ws = new FakeClientSocket();
    startSession(ws);
    expect(ws.sent).toEqual([{ type: "session", state: "ready" }]);
  });

  it("coalesces joystick frames and forwards at most once per flush interval", () => {
    const ws = new FakeClientSocket();
    startSession(ws);

    ws.receiveFromClient({ type: "joystick", seq: 1, data: { linear: 0.2, angular: 0 } });
    ws.receiveFromClient({ type: "joystick", seq: 2, data: { linear: 0.3, angular: 0.1 } });

    jest.advanceTimersByTime(40);

    const velocityFrames = registry.sentFrames.filter((f) => f.frame.type === "teleop.velocity");
    expect(velocityFrames).toHaveLength(1);
    expect(velocityFrames[0].frame).toMatchObject({ type: "teleop.velocity", data: { linear: 0.3, angular: 0.1 } });
  });

  it("forwards joystick values unclamped — RobotServer's TeleopService is the sole clamp authority", () => {
    const ws = new FakeClientSocket();
    startSession(ws);

    ws.receiveFromClient({ type: "joystick", seq: 1, data: { linear: 999, angular: -999 } });
    jest.advanceTimersByTime(40);

    const [sent] = registry.sentFrames.filter((f) => f.frame.type === "teleop.velocity");
    expect(sent.frame).toMatchObject({ type: "teleop.velocity", data: { linear: 999, angular: -999 } });
  });

  it("forwards estop frames immediately and acks the client", () => {
    const ws = new FakeClientSocket();
    startSession(ws);

    ws.receiveFromClient({ type: "estop", data: { engaged: true } });

    expect(registry.sentFrames).toEqual([
      { robotId: robotSlug, frame: { type: "teleop.estop", sessionId: expect.any(String), engaged: true } },
    ]);
    expect(ws.sent).toContainEqual({ type: "estop_ack", engaged: true });
  });

  it("a newer session for the same robot preempts the older one", () => {
    const first = new FakeClientSocket();
    startSession(first);

    const second = new FakeClientSocket();
    startSession(second);

    expect(first.sent).toContainEqual({ type: "session", state: "preempted" });
    expect(first.closed).toBe(true);
    expect(second.sent).toContainEqual({ type: "session", state: "ready" });
    expect(second.closed).toBe(false);
  });

  it("sends teleop.release to the robot when the client disconnects", () => {
    const ws = new FakeClientSocket();
    startSession(ws);
    ws.receiveFromClient({ type: "joystick", seq: 1, data: { linear: 0.3, angular: 0 } });

    ws.close();

    const releaseFrames = registry.sentFrames.filter((f) => f.frame.type === "teleop.release");
    expect(releaseFrames).toHaveLength(1);
  });

  it("revokes the session when the assignment's platformAccess is later revoked", async () => {
    const ws = new FakeClientSocket();
    startSession(ws);

    const assignment = await assignmentRepository.findActiveByUserAndRobot(userId, robotMongoId);
    await assignmentRepository.updateAccess(assignment!._id.toString(), { platformAccess: false });

    // Revalidation runs on a 30s interval — advance past it and let the pending promise settle.
    await jest.advanceTimersByTimeAsync(30_001);

    expect(ws.sent).toContainEqual({ type: "session", state: "revoked" });
    expect(ws.closed).toBe(true);
  });

  it("notifies the client and skips teleop.release when the robot's link goes offline mid-session", () => {
    const ws = new FakeClientSocket();
    startSession(ws);

    registry.simulateDisconnect(robotSlug);

    expect(ws.sent).toContainEqual({ type: "session", state: "robot_offline" });
    expect(ws.closed).toBe(true);
    expect(registry.sentFrames.filter((f) => f.frame.type === "teleop.release")).toHaveLength(0);
  });
});
