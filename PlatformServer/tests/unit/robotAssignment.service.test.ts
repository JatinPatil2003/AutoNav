import { RobotAssignmentService } from "../../src/modules/robot-assignments/robotAssignment.service";
import { ConflictError } from "../../src/common/errors";
import { FakeUserRepository } from "./fakes/FakeUserRepository";
import { FakeRobotRepository } from "./fakes/FakeRobotRepository";
import { FakeRobotAssignmentRepository } from "./fakes/FakeRobotAssignmentRepository";
import { FakeRobotNotifier } from "./fakes/FakeRobotNotifier";

describe("RobotAssignmentService", () => {
  let service: RobotAssignmentService;
  let userRepository: FakeUserRepository;
  let robotRepository: FakeRobotRepository;
  let notifier: FakeRobotNotifier;
  let robotId: string;
  let adminId: string;

  beforeEach(async () => {
    userRepository = new FakeUserRepository();
    robotRepository = new FakeRobotRepository();
    notifier = new FakeRobotNotifier();
    service = new RobotAssignmentService(
      new FakeRobotAssignmentRepository(robotRepository, userRepository),
      robotRepository,
      notifier,
    );

    const robot = await robotRepository.create({
      robotId: "bot-1",
      name: "Bot 1",
      vpnHostname: "bot-1.robot",
      robotSecretEncrypted: { ciphertext: "x", iv: "x", authTag: "x" },
      robotSecretHash: "x",
    });
    robotId = robot._id.toString();

    const admin = await userRepository.create({
      email: "admin@example.com",
      name: "Admin",
      role: "admin",
      authProviders: ["local"],
    });
    adminId = admin._id.toString();
  });

  it("grants a user access to a robot, and it shows up in their active robots", async () => {
    const user = await userRepository.create({
      email: "u1@example.com",
      name: "U1",
      role: "user",
      authProviders: ["local"],
    });

    await service.grant(user._id.toString(), robotId, adminId);

    const robots = await service.listActiveRobotsForUser(user._id.toString());
    expect(robots).toHaveLength(1);
    expect(robots[0].id).toBe(robotId);
  });

  it("supports one robot having many users and one user having many robots", async () => {
    const userA = await userRepository.create({
      email: "a@example.com",
      name: "A",
      role: "user",
      authProviders: ["local"],
    });
    const userB = await userRepository.create({
      email: "b@example.com",
      name: "B",
      role: "user",
      authProviders: ["local"],
    });
    const otherRobot = await robotRepository.create({
      robotId: "bot-2",
      name: "Bot 2",
      vpnHostname: "bot-2.robot",
      robotSecretEncrypted: { ciphertext: "x", iv: "x", authTag: "x" },
      robotSecretHash: "x",
    });

    await service.grant(userA._id.toString(), robotId, adminId);
    await service.grant(userB._id.toString(), robotId, adminId);
    await service.grant(userA._id.toString(), otherRobot._id.toString(), adminId);

    expect(await service.listActiveRobotsForUser(userA._id.toString())).toHaveLength(2);
    expect(await service.listActiveRobotsForUser(userB._id.toString())).toHaveLength(1);
    expect(await service.listForRobot(robotId)).toHaveLength(2);
  });

  it("rejects granting the same active (user, robot) pair twice", async () => {
    const user = await userRepository.create({
      email: "u2@example.com",
      name: "U2",
      role: "user",
      authProviders: ["local"],
    });
    await service.grant(user._id.toString(), robotId, adminId);

    await expect(service.grant(user._id.toString(), robotId, adminId)).rejects.toBeInstanceOf(
      ConflictError,
    );
  });

  it("revoking removes the robot from the user's active list and notifies the robot", async () => {
    const user = await userRepository.create({
      email: "u3@example.com",
      name: "U3",
      role: "user",
      authProviders: ["local"],
    });
    const assignment = await service.grant(user._id.toString(), robotId, adminId);

    await service.revoke(assignment.id, adminId, "no longer needed");

    expect(await service.listActiveRobotsForUser(user._id.toString())).toEqual([]);
    expect(notifier.notified).toContain("bot-1");
  });

  it("re-granting after a revoke reactivates the same assignment (not a duplicate)", async () => {
    const user = await userRepository.create({
      email: "u4@example.com",
      name: "U4",
      role: "user",
      authProviders: ["local"],
    });
    const assignment = await service.grant(user._id.toString(), robotId, adminId);
    await service.revoke(assignment.id, adminId);

    const regranted = await service.grant(user._id.toString(), robotId, adminId);

    expect(regranted.id).toBe(assignment.id);
    expect(await service.listActiveRobotsForUser(user._id.toString())).toHaveLength(1);
  });
});
