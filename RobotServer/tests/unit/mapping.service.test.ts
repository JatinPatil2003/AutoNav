import { MappingService } from "../../src/modules/mapping/mapping.service";
import { LaunchManager } from "../../src/ros/launchManager";
import { RosUnavailableError } from "../../src/common/errors";
import { FakeRosTopicGateway } from "./fakes/FakeRosTopicGateway";
import { FakeGoalRepository } from "./fakes/FakeGoalRepository";
import { FakeMapRepository } from "./fakes/FakeMapRepository";
import { makePoseMessage } from "./helpers/poseFixture";

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
});
