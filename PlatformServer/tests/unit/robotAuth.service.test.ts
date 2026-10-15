import { RobotAuthService } from "../../src/modules/robot-auth/robotAuth.service";
import { RobotAssignmentService } from "../../src/modules/robot-assignments/robotAssignment.service";
import { encryptSecret, hashSecretForLookup } from "../../src/common/utils/secretCrypto.util";
import { UnauthorizedError } from "../../src/common/errors";
import { FakeUserRepository } from "./fakes/FakeUserRepository";
import { FakeRobotRepository } from "./fakes/FakeRobotRepository";
import { FakeRobotAssignmentRepository } from "./fakes/FakeRobotAssignmentRepository";
import { FakeRobotNotifier } from "./fakes/FakeRobotNotifier";
import { FakeEmailOtpRepository } from "./fakes/FakeEmailOtpRepository";
import { FakeEmailOtpSender } from "./fakes/FakeEmailOtpSender";

describe("RobotAuthService", () => {
  let service: RobotAuthService;
  let userRepository: FakeUserRepository;
  let robotRepository: FakeRobotRepository;
  let assignmentRepository: FakeRobotAssignmentRepository;
  let assignmentService: RobotAssignmentService;
  let otpRepository: FakeEmailOtpRepository;
  let emailOtpSender: FakeEmailOtpSender;
  let robotId: string;
  let robotMongoId: string;

  beforeEach(async () => {
    userRepository = new FakeUserRepository();
    robotRepository = new FakeRobotRepository();
    assignmentRepository = new FakeRobotAssignmentRepository(robotRepository, userRepository);
    assignmentService = new RobotAssignmentService(
      assignmentRepository,
      robotRepository,
      new FakeRobotNotifier(),
    );
    otpRepository = new FakeEmailOtpRepository();
    emailOtpSender = new FakeEmailOtpSender();
    service = new RobotAuthService(assignmentRepository, userRepository, otpRepository, emailOtpSender);

    const robot = await robotRepository.create({
      robotId: "bot-1",
      name: "Bot 1",
      vpnHostname: "bot-1.robot",
      robotSecretEncrypted: encryptSecret("correct-secret"),
      robotSecretHash: hashSecretForLookup("correct-secret"),
    });
    robotId = robot.robotId;
    robotMongoId = robot._id.toString();
  });

  it("completes the full pin-setup OTP relay and roster reflects the new pin", async () => {
    const user = await userRepository.create({
      email: "pinuser@example.com",
      name: "Pin User",
      role: "user",
      authProviders: ["local"],
    });
    const admin = await userRepository.create({
      email: "admin@example.com",
      name: "Admin",
      role: "admin",
      authProviders: ["local"],
    });
    await assignmentService.grant(user._id.toString(), robotMongoId, admin._id.toString());

    await service.requestPinSetupOtp(robotMongoId, "pinuser@example.com");
    expect(emailOtpSender.sent).toHaveLength(1);
    const otp = emailOtpSender.sent[0].otp;

    const { pinSetupToken } = await service.verifyPinSetupOtp("pinuser@example.com", otp, robotId);
    const entry = await service.confirmPinSetup(pinSetupToken, "1234", robotId, robotMongoId);

    expect(entry.pinSet).toBe(true);
    expect(entry.email).toBe("pinuser@example.com");

    const roster = await service.getRoster(robotMongoId);
    expect(roster).toHaveLength(1);
    expect(roster[0].pinSet).toBe(true);
  });

  it("never sends an OTP for a user with no active assignment on this robot (anti-enumeration)", async () => {
    await userRepository.create({
      email: "noaccess@example.com",
      name: "No Access",
      role: "user",
      authProviders: ["local"],
    });

    await service.requestPinSetupOtp(robotMongoId, "noaccess@example.com");
    expect(emailOtpSender.sent).toHaveLength(0);
  });

  it("rejects a pinSetupToken when replayed against a different robot", async () => {
    const user = await userRepository.create({
      email: "crossbot@example.com",
      name: "Cross Bot",
      role: "user",
      authProviders: ["local"],
    });
    const admin = await userRepository.create({
      email: "admin2@example.com",
      name: "Admin2",
      role: "admin",
      authProviders: ["local"],
    });
    const otherRobot = await robotRepository.create({
      robotId: "bot-2",
      name: "Bot 2",
      vpnHostname: "bot-2.robot",
      robotSecretEncrypted: encryptSecret("bot-2-secret"),
      robotSecretHash: hashSecretForLookup("bot-2-secret"),
    });
    await assignmentService.grant(user._id.toString(), robotMongoId, admin._id.toString());

    await service.requestPinSetupOtp(robotMongoId, "crossbot@example.com");
    const otp = emailOtpSender.sent[0].otp;
    const { pinSetupToken } = await service.verifyPinSetupOtp("crossbot@example.com", otp, robotId);

    await expect(
      service.confirmPinSetup(pinSetupToken, "1234", otherRobot.robotId, otherRobot._id.toString()),
    ).rejects.toBeInstanceOf(UnauthorizedError);
  });
});
