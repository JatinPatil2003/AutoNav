import { RobotAccessRequestService } from "../../src/modules/robot-access-requests/robotAccessRequest.service";
import { ConflictError } from "../../src/common/errors";
import { FakeUserRepository } from "./fakes/FakeUserRepository";
import { FakeRobotAccessRequestRepository } from "./fakes/FakeRobotAccessRequestRepository";

describe("RobotAccessRequestService", () => {
  let service: RobotAccessRequestService;
  let userRepository: FakeUserRepository;
  let requestRepository: FakeRobotAccessRequestRepository;

  beforeEach(() => {
    userRepository = new FakeUserRepository();
    requestRepository = new FakeRobotAccessRequestRepository();
    service = new RobotAccessRequestService(requestRepository, userRepository);
  });

  it("creates a pending request for a robotless user", async () => {
    const user = await userRepository.create({
      email: "u1@example.com",
      name: "U1",
      role: "user",
      authProviders: ["local"],
    });

    const request = await service.createRequest(user._id.toString());

    expect(request.status).toBe("pending");
  });

  it("rejects a second simultaneous pending request", async () => {
    const user = await userRepository.create({
      email: "u2@example.com",
      name: "U2",
      role: "user",
      authProviders: ["local"],
    });

    await service.createRequest(user._id.toString());

    await expect(service.createRequest(user._id.toString())).rejects.toBeInstanceOf(ConflictError);
  });

  it("rejects a request from a user who already has a robot assigned", async () => {
    const user = await userRepository.create({
      email: "u3@example.com",
      name: "U3",
      role: "user",
      authProviders: ["local"],
    });
    await userRepository.assignRobot(user._id.toString(), "507f1f77bcf86cd799439011");

    await expect(service.createRequest(user._id.toString())).rejects.toBeInstanceOf(ConflictError);
  });
});
