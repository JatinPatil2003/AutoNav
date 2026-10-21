import { MappingService } from "../../src/modules/mapping/mapping.service";
import { SLAM_LAUNCH_FILE } from "../../src/modules/mapping/mapping.types";
import { LaunchManager } from "../../src/ros/launchManager";
import { RosUnavailableError } from "../../src/common/errors";
import { FakeRosTopicGateway } from "./fakes/FakeRosTopicGateway";
import { FakeGoalRepository } from "./fakes/FakeGoalRepository";
import { FakeMapRepository } from "./fakes/FakeMapRepository";
import { makePoseMessage, makePoseStampedMessage } from "./helpers/poseFixture";

describe("MappingService", () => {
  let topicGateway: FakeRosTopicGateway;
  let goalRepository: FakeGoalRepository;
  let mapRepository: FakeMapRepository;
  let service: MappingService;

  beforeEach(() => {
    topicGateway = new FakeRosTopicGateway();
    goalRepository = new FakeGoalRepository();
    mapRepository = new FakeMapRepository();
    service = new MappingService(topicGateway, goalRepository, mapRepository, new LaunchManager(), "/tmp/maps");
  });

  it("refuses to capture a waypoint with no live pose", async () => {
    await expect(service.captureWaypoint("doorway")).rejects.toBeInstanceOf(RosUnavailableError);
  });

  it("captures a waypoint under the temp map when a pose is available", async () => {
    topicGateway.setLatest("/amcl_pose", makePoseMessage({ x: 1, y: 2, theta: 0.1 }));
    await service.captureWaypoint("doorway");
    const saved = await goalRepository.findOne("temp", "doorway");
    expect(saved?.x).toBeCloseTo(1);
    expect(saved?.y).toBeCloseTo(2);
    expect(saved?.theta).toBeCloseTo(0.1);
  });

  it("commits temp waypoints onto the named map and clears temp", async () => {
    await goalRepository.upsert("temp", "doorway", { x: 1, y: 2, theta: 0.1 });

    const processed = await service.commitWaypoints("cafe");

    expect(processed).toBe(1);
    await expect(goalRepository.findOne("cafe", "doorway")).resolves.toEqual({ x: 1, y: 2, theta: 0.1 });
    await expect(goalRepository.listNames("temp")).resolves.toEqual([]);
  });

  it("maps every SlamBackend to a distinct autonav_navigation launch file", () => {
    expect(SLAM_LAUNCH_FILE.cartographer).toBe("cartographer.launch.py");
    expect(SLAM_LAUNCH_FILE.slam_toolbox).toBe("slam.launch.py");
  });

  it("starts mapping (Cartographer by default) without throwing", async () => {
    await expect(service.start()).resolves.toBeUndefined();
  });

  it("accepts an explicit slam_toolbox backend without throwing", async () => {
    await expect(service.start("slam_toolbox")).resolves.toBeUndefined();
  });

  it("prefers the SLAM bridge pose over AMCL while mapping is active", async () => {
    await service.start();
    topicGateway.setLatest("/amcl_pose", makePoseMessage({ x: 9, y: 9, theta: 0 }));
    topicGateway.setLatest("/pose_bridge/map_pose", makePoseStampedMessage({ x: 1, y: 2, theta: 0.1 }));

    const location = service.getCurrentLocation();

    expect(location?.x).toBeCloseTo(1);
    expect(location?.y).toBeCloseTo(2);
    expect(location?.theta).toBeCloseTo(0.1);
  });

  it("falls back to AMCL while mapping is active if the bridge hasn't published yet", async () => {
    await service.start();
    topicGateway.setLatest("/amcl_pose", makePoseMessage({ x: 3, y: 4, theta: 0.2 }));

    const location = service.getCurrentLocation();

    expect(location?.x).toBeCloseTo(3);
    expect(location?.y).toBeCloseTo(4);
  });

  it("ignores a stale bridge pose once mapping has stopped, using AMCL instead", async () => {
    await service.start();
    topicGateway.setLatest("/pose_bridge/map_pose", makePoseStampedMessage({ x: 1, y: 2, theta: 0.1 }));
    service.stop();
    topicGateway.setLatest("/amcl_pose", makePoseMessage({ x: 5, y: 6, theta: 0 }));

    const location = service.getCurrentLocation();

    expect(location?.x).toBeCloseTo(5);
    expect(location?.y).toBeCloseTo(6);
  });
});
