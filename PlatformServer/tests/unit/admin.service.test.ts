import { AdminService } from "../../src/modules/admin/admin.service";
import { RobotAssignmentService } from "../../src/modules/robot-assignments/robotAssignment.service";
import { ConflictError } from "../../src/common/errors";
import { FakeUserRepository } from "./fakes/FakeUserRepository";
import { FakeRobotRepository } from "./fakes/FakeRobotRepository";
import { FakeRobotAssignmentRepository } from "./fakes/FakeRobotAssignmentRepository";
import { FakeRobotAccessRequestRepository } from "./fakes/FakeRobotAccessRequestRepository";
import { FakeRobotNotifier } from "./fakes/FakeRobotNotifier";

describe("AdminService", () => {
  let adminService: AdminService;
  let userRepository: FakeUserRepository;
  let robotRepository: FakeRobotRepository;
  let requestRepository: FakeRobotAccessRequestRepository;
  let robotAssignmentService: RobotAssignmentService;
  let adminId: string;
  let robotId: string;
  let otherRobotId: string;

  beforeEach(async () => {
    userRepository = new FakeUserRepository();
    robotRepository = new FakeRobotRepository();
    requestRepository = new FakeRobotAccessRequestRepository();
    robotAssignmentService = new RobotAssignmentService(
      new FakeRobotAssignmentRepository(robotRepository, userRepository),
      robotRepository,
      new FakeRobotNotifier(),
    );
    adminService = new AdminService(requestRepository, userRepository, robotAssignmentService);

    const admin = await userRepository.create({
      email: "admin@example.com",
      name: "Admin",
      role: "admin",
      authProviders: ["local"],
    });
    adminId = admin._id.toString();

    const robot = await robotRepository.create({
      robotId: "bot-1",
      name: "Bot 1",
      vpnHostname: "bot-1.robot",
      robotSecretEncrypted: { ciphertext: "x", iv: "x", authTag: "x" },
      robotSecretHash: "x",
    });
    robotId = robot._id.toString();

    const otherRobot = await robotRepository.create({
      robotId: "bot-2",
      name: "Bot 2",
      vpnHostname: "bot-2.robot",
      robotSecretEncrypted: { ciphertext: "x", iv: "x", authTag: "x" },
      robotSecretHash: "x",
    });
    otherRobotId = otherRobot._id.toString();
  });

  it("approving a request grants access to the originally requested robot by default", async () => {
    const user = await userRepository.create({
      email: "u1@example.com",
      name: "U1",
      role: "user",
      authProviders: ["local"],
    });
    const request = await requestRepository.create(user._id.toString(), robotId);

    await adminService.approveRequest(request._id.toString(), adminId);

    const robots = await robotAssignmentService.listActiveRobotsForUser(user._id.toString());
    expect(robots.map((r) => r.id)).toEqual([robotId]);
  });

  it("an admin can override the request's robot at approval time", async () => {
    const user = await userRepository.create({
      email: "u2@example.com",
      name: "U2",
      role: "user",
      authProviders: ["local"],
    });
    const request = await requestRepository.create(user._id.toString(), robotId);

    await adminService.approveRequest(request._id.toString(), adminId, otherRobotId);

    const robots = await robotAssignmentService.listActiveRobotsForUser(user._id.toString());
    expect(robots.map((r) => r.id)).toEqual([otherRobotId]);
  });

  it("rejects approving an already-approved request", async () => {
    const user = await userRepository.create({
      email: "u3@example.com",
      name: "U3",
      role: "user",
      authProviders: ["local"],
    });
    const request = await requestRepository.create(user._id.toString(), robotId);
    await adminService.approveRequest(request._id.toString(), adminId);

    await expect(adminService.approveRequest(request._id.toString(), adminId)).rejects.toBeInstanceOf(
      ConflictError,
    );
  });
});
