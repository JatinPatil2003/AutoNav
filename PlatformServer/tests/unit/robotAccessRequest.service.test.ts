import { Types } from "mongoose";
import { RobotAccessRequestService } from "../../src/modules/robot-access-requests/robotAccessRequest.service";
import { RobotAssignmentService } from "../../src/modules/robot-assignments/robotAssignment.service";
import { ConflictError, NotFoundError } from "../../src/common/errors";
import { FakeUserRepository } from "./fakes/FakeUserRepository";
import { FakeRobotRepository } from "./fakes/FakeRobotRepository";
import { FakeRobotAccessRequestRepository } from "./fakes/FakeRobotAccessRequestRepository";
import { FakeRobotAssignmentRepository } from "./fakes/FakeRobotAssignmentRepository";
import { FakeRobotNotifier } from "./fakes/FakeRobotNotifier";

describe("RobotAccessRequestService", () => {
  let service: RobotAccessRequestService;
  let userRepository: FakeUserRepository;
  let robotRepository: FakeRobotRepository;
  let requestRepository: FakeRobotAccessRequestRepository;
  let robotAssignmentService: RobotAssignmentService;
  let robotId: string;

  beforeEach(async () => {
    userRepository = new FakeUserRepository();
    robotRepository = new FakeRobotRepository();
    requestRepository = new FakeRobotAccessRequestRepository();
    robotAssignmentService = new RobotAssignmentService(
      new FakeRobotAssignmentRepository(),
      robotRepository,
      new FakeRobotNotifier(),
    );
    service = new RobotAccessRequestService(requestRepository, robotRepository, robotAssignmentService);

    const robot = await robotRepository.create({
      robotId: "test-bot",
      name: "Test Bot",
      vpnHostname: "test-bot.robot",
    });
    robotId = robot._id.toString();
  });

  it("creates a pending request for a robotless user", async () => {
    const user = await userRepository.create({
      email: "u1@example.com",
      name: "U1",
      role: "user",
      authProviders: ["local"],
    });

    const request = await service.createRequest(user._id.toString(), robotId);

    expect(request.status).toBe("pending");
  });

  it("rejects a second simultaneous pending request for the same robot", async () => {
    const user = await userRepository.create({
      email: "u2@example.com",
      name: "U2",
      role: "user",
      authProviders: ["local"],
    });

    await service.createRequest(user._id.toString(), robotId);

    await expect(service.createRequest(user._id.toString(), robotId)).rejects.toBeInstanceOf(
      ConflictError,
    );
  });

  it("allows concurrent pending requests for different robots", async () => {
    const user = await userRepository.create({
      email: "u2b@example.com",
      name: "U2b",
      role: "user",
      authProviders: ["local"],
    });
    const otherRobot = await robotRepository.create({
      robotId: "test-bot-2",
      name: "Test Bot 2",
      vpnHostname: "test-bot-2.robot",
    });

    await service.createRequest(user._id.toString(), robotId);
    await expect(
      service.createRequest(user._id.toString(), otherRobot._id.toString()),
    ).resolves.toBeDefined();
  });

  it("rejects a request for a robot the user already has active access to", async () => {
    const user = await userRepository.create({
      email: "u3@example.com",
      name: "U3",
      role: "user",
      authProviders: ["local"],
    });
    const admin = await userRepository.create({
      email: "admin@example.com",
      name: "Admin",
      role: "admin",
      authProviders: ["local"],
    });
    await robotAssignmentService.grant(user._id.toString(), robotId, admin._id.toString());

    await expect(service.createRequest(user._id.toString(), robotId)).rejects.toBeInstanceOf(
      ConflictError,
    );
  });

  it("rejects a request for a robot that does not exist", async () => {
    const user = await userRepository.create({
      email: "u4@example.com",
      name: "U4",
      role: "user",
      authProviders: ["local"],
    });

    await expect(
      service.createRequest(user._id.toString(), new Types.ObjectId().toString()),
    ).rejects.toBeInstanceOf(NotFoundError);
  });
});
