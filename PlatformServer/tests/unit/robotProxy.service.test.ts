import { NotFoundError, RobotUnreachableError } from "../../src/common/errors";
import { RobotProxyService } from "../../src/modules/robot-proxy/robotProxy.service";
import { FakeRobotApiClient } from "./fakes/FakeRobotApiClient";
import { FakeRobotRepository } from "./fakes/FakeRobotRepository";

describe("RobotProxyService", () => {
  let robotRepository: FakeRobotRepository;
  let apiClient: FakeRobotApiClient;
  let service: RobotProxyService;
  let robotMongoId: string;
  const robotSlug = "bot-1";

  beforeEach(async () => {
    robotRepository = new FakeRobotRepository();
    apiClient = new FakeRobotApiClient();
    service = new RobotProxyService(robotRepository, apiClient);

    const robot = await robotRepository.create({
      robotId: robotSlug,
      name: "Bot One",
      vpnHostname: "bot-1.vpn.internal",
      displayName: "Bot One",
    });
    robotMongoId = robot._id.toString();
  });

  it("rejects an unknown robot before ever calling the robot's API", async () => {
    await expect(service.startMapping("000000000000000000000000")).rejects.toBeInstanceOf(NotFoundError);
    expect(apiClient.calls).toHaveLength(0);
  });

  it("starts mapping with an explicit backend", async () => {
    await service.startMapping(robotMongoId, "slam_toolbox");

    expect(apiClient.calls).toEqual([
      { robotId: robotSlug, method: "POST", path: "/api/mapping/start", body: { backend: "slam_toolbox" } },
    ]);
  });

  it("starts mapping with no backend specified, leaving RobotServer to apply its own default", async () => {
    await service.startMapping(robotMongoId);

    expect(apiClient.calls[0].body).toEqual({});
  });

  it("proxies a list-maps request to the robot's /api/navigation/maps", async () => {
    apiClient.response = { maps: ["cafe", "warehouse"] };

    const result = await service.listMaps(robotMongoId);

    expect(result).toEqual({ maps: ["cafe", "warehouse"] });
    expect(apiClient.calls).toEqual([{ robotId: robotSlug, method: "GET", path: "/api/navigation/maps", body: undefined }]);
  });

  it("proxies waypoint capture to the robot's /api/mapping/waypoints", async () => {
    await service.captureWaypoint(robotMongoId, "doorway");

    expect(apiClient.calls).toEqual([
      { robotId: robotSlug, method: "POST", path: "/api/mapping/waypoints", body: { name: "doorway" } },
    ]);
  });

  it("proxies a goal start command with mapName and name", async () => {
    await service.startGoal(robotMongoId, "cafe", "entrance");

    expect(apiClient.calls).toEqual([
      { robotId: robotSlug, method: "POST", path: "/api/navigation/goal/start", body: { mapName: "cafe", name: "entrance" } },
    ]);
  });

  it("propagates RobotUnreachableError when the robot's API call fails", async () => {
    apiClient.shouldFail = true;

    await expect(service.stopMapping(robotMongoId)).rejects.toBeInstanceOf(RobotUnreachableError);
  });
});
