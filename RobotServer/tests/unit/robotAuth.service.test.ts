import bcrypt from "bcryptjs";
import { RobotAuthService } from "../../src/modules/robot-auth/robotAuth.service";
import { RosterSyncService } from "../../src/modules/robot-auth/rosterSync.service";
import { DeviceIdentityService } from "../../src/ros/deviceIdentity";
import { UnauthorizedError } from "../../src/common/errors";
import { FakeLocalUserRepository } from "./fakes/FakeLocalUserRepository";
import { FakePlatformClient } from "./fakes/FakePlatformClient";
import { FakeRosServiceGateway } from "./fakes/FakeRosServiceGateway";

const TEST_ROBOT_ID = "test-robot-01";

describe("RobotAuthService", () => {
  let localUserRepository: FakeLocalUserRepository;
  let platformClient: FakePlatformClient;
  let deviceIdentity: DeviceIdentityService;
  let service: RobotAuthService;
  let rosterSync: RosterSyncService;

  beforeEach(async () => {
    const serviceGateway = new FakeRosServiceGateway();
    serviceGateway.setResponse("/esp32_bridge_node/get_device_id", {
      success: true,
      message: TEST_ROBOT_ID,
    });
    deviceIdentity = new DeviceIdentityService(serviceGateway);
    await deviceIdentity.resolve();

    localUserRepository = new FakeLocalUserRepository();
    platformClient = new FakePlatformClient();
    service = new RobotAuthService(localUserRepository, platformClient, deviceIdentity);
    rosterSync = new RosterSyncService(platformClient, localUserRepository, deviceIdentity);
  });

  it("identify reports pinSet=false for a user with no pin cached yet", async () => {
    platformClient.roster = [
      {
        assignmentId: "a1",
        userId: "u1",
        email: "u1@example.com",
        name: "U1",
        pinHash: null,
        pinSet: false,
        pinUpdatedCount: 0,
      },
    ];
    await rosterSync.sync();

    const result = await service.identify("u1@example.com");
    expect(result).toEqual({ status: "has-access", pinSet: false, name: "U1" });
  });

  it("reports no-account for an email with no PlatformServer account at all", async () => {
    const result = await service.identify("nobody@example.com");
    expect(result).toEqual({ status: "no-account", pinSet: false });
  });

  it("reports no-access for an email with a PlatformServer account but no assignment on this robot", async () => {
    platformClient.accountsWithoutAccess.add("unassigned@example.com");
    const result = await service.identify("unassigned@example.com");
    expect(result).toEqual({ status: "no-access", pinSet: false });
  });

  it("logs in with the correct cached PIN", async () => {
    const pinHash = await bcrypt.hash("1234", 12);
    platformClient.roster = [
      {
        assignmentId: "a1",
        userId: "u1",
        email: "pinuser@example.com",
        name: "U1",
        pinHash,
        pinSet: true,
        pinUpdatedCount: 1,
      },
    ];
    await rosterSync.sync();

    const tokens = await service.loginWithPin("pinuser@example.com", "1234");
    expect(tokens.accessToken).toEqual(expect.any(String));
    expect(tokens.refreshToken).toEqual(expect.any(String));
  });

  it("rejects a wrong PIN and locks out after too many attempts", async () => {
    const pinHash = await bcrypt.hash("1234", 12);
    platformClient.roster = [
      {
        assignmentId: "a1",
        userId: "u2",
        email: "lockout@example.com",
        name: "U2",
        pinHash,
        pinSet: true,
        pinUpdatedCount: 1,
      },
    ];
    await rosterSync.sync();

    for (let i = 0; i < 5; i += 1) {
      await expect(service.loginWithPin("lockout@example.com", "0000")).rejects.toBeInstanceOf(
        UnauthorizedError,
      );
    }

    // Even the *correct* PIN is now rejected until the lockout window passes.
    await expect(service.loginWithPin("lockout@example.com", "1234")).rejects.toBeInstanceOf(
      UnauthorizedError,
    );
  });

  it("completes the pin-setup flow without auto-logging in", async () => {
    platformClient.roster = [
      {
        assignmentId: "a1",
        userId: "u3",
        email: "newuser@example.com",
        name: "U3",
        pinHash: null,
        pinSet: false,
        pinUpdatedCount: 0,
      },
    ];
    await rosterSync.sync();

    await service.requestPinSetupOtp("newuser@example.com");
    platformClient.seedOtp("newuser@example.com", "654321");

    const { pinSetupToken } = await service.verifyPinSetupOtp("newuser@example.com", "654321");
    const result = await service.confirmPinSetup(pinSetupToken, "9876");

    expect(result.email).toBe("newuser@example.com");

    const identified = await service.identify("newuser@example.com");
    expect(identified.pinSet).toBe(true);

    const loginTokens = await service.loginWithPin("newuser@example.com", "9876");
    expect(loginTokens.accessToken).toEqual(expect.any(String));
  });

  it("rejects login for a user who no longer has active access (revoked by the roster)", async () => {
    const pinHash = await bcrypt.hash("1234", 12);
    platformClient.roster = [
      {
        assignmentId: "a1",
        userId: "u4",
        email: "revoked@example.com",
        name: "U4",
        pinHash,
        pinSet: true,
        pinUpdatedCount: 1,
      },
    ];
    await rosterSync.sync();

    platformClient.roster = [];
    await rosterSync.sync();

    await expect(service.loginWithPin("revoked@example.com", "1234")).rejects.toBeInstanceOf(
      UnauthorizedError,
    );
  });
});
