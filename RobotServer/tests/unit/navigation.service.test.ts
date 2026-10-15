import { NavigationService } from "../../src/modules/navigation/navigation.service";
import { NavStatus } from "../../src/modules/navigation/navigation.types";
import { TeleopService } from "../../src/modules/teleop/teleop.service";
import { LaunchManager } from "../../src/ros/launchManager";
import { NotFoundError } from "../../src/common/errors";
import { FakeRosActionGateway } from "./fakes/FakeRosActionGateway";
import { FakeRosServiceGateway } from "./fakes/FakeRosServiceGateway";
import { FakeRosTopicGateway } from "./fakes/FakeRosTopicGateway";
import { FakeGoalRepository } from "./fakes/FakeGoalRepository";
import { FakeMapRepository } from "./fakes/FakeMapRepository";
import { makePoseMessage } from "./helpers/poseFixture";

const SET_INITIAL_POSE_SERVICE = "/set_initial_pose";

// Real rotation time for TeleopService.rotateInPlace(2) at ANGULAR_SPEED=0.6 rad/s.
const ROTATE_IN_PLACE_MS = 2 * ((2 * Math.PI + Math.PI / 3) / 0.6) * 1000;

describe("NavigationService", () => {
  let actionGateway: FakeRosActionGateway;
  let serviceGateway: FakeRosServiceGateway;
  let topicGateway: FakeRosTopicGateway;
  let teleopService: TeleopService;
  let goalRepository: FakeGoalRepository;
  let mapRepository: FakeMapRepository;
  let service: NavigationService;

  beforeEach(() => {
    actionGateway = new FakeRosActionGateway();
    serviceGateway = new FakeRosServiceGateway();
    topicGateway = new FakeRosTopicGateway();
    teleopService = new TeleopService(topicGateway);
    goalRepository = new FakeGoalRepository();
    mapRepository = new FakeMapRepository();
    service = new NavigationService(
      actionGateway,
      serviceGateway,
      topicGateway,
      teleopService,
      goalRepository,
      mapRepository,
      new LaunchManager(),
      "/tmp/maps",
    );
  });

  it("rejects starting navigation on a map that doesn't exist", async () => {
    await expect(service.start("missing-map")).rejects.toBeInstanceOf(NotFoundError);
  });

  it("starts navigation once the map has been loaded", async () => {
    mapRepository.existingMaps.add("cafe");
    await expect(service.start("cafe")).resolves.toBeUndefined();
  });

  it("categorizes goals by name", async () => {
    await goalRepository.save("cafe", "localization_1", { x: 0, y: 0, theta: 0 });
    await goalRepository.save("cafe", "charger_1", { x: 1, y: 0, theta: 0 });
    await goalRepository.save("cafe", "standby_1", { x: 2, y: 0, theta: 0 });
    await goalRepository.save("cafe", "goal_1", { x: 3, y: 0, theta: 0 });

    const categorized = await service.listGoals("cafe");

    expect(categorized).toEqual({
      localization: ["localization_1"],
      charging: ["charger_1"],
      standby: ["standby_1"],
      goals: ["goal_1"],
    });
  });

  it("throws NotFoundError for a goal that was never saved", async () => {
    await expect(service.getGoal("cafe", "nope")).rejects.toBeInstanceOf(NotFoundError);
  });

  it("reports localization success when AMCL is confident", async () => {
    await goalRepository.save("cafe", "localization_1", { x: 1, y: 2, theta: 0.5 });
    topicGateway.setLatest("/amcl_pose", makePoseMessage({ x: 1, y: 2, theta: 0.5, confident: true }));

    jest.useFakeTimers();
    try {
      const resultPromise = service.localize("cafe", "localization_1");
      await jest.advanceTimersByTimeAsync(ROTATE_IN_PLACE_MS + 1000);
      const result = await resultPromise;

      expect(result.success).toBe(true);
      expect(serviceGateway.calls[0]).toMatchObject({
        serviceName: SET_INITIAL_POSE_SERVICE,
      });
    } finally {
      jest.useRealTimers();
    }
  });

  it("reports localization failure when AMCL is not confident", async () => {
    await goalRepository.save("cafe", "localization_1", { x: 1, y: 2, theta: 0.5 });
    topicGateway.setLatest("/amcl_pose", makePoseMessage({ x: 1, y: 2, theta: 0.5, confident: false }));

    jest.useFakeTimers();
    try {
      const resultPromise = service.localize("cafe", "localization_1");
      await jest.advanceTimersByTimeAsync(ROTATE_IN_PLACE_MS + 1000);
      const result = await resultPromise;

      expect(result.success).toBe(false);
    } finally {
      jest.useRealTimers();
    }
  });

  it("sends and cancels navigation goals through the gateway", async () => {
    await goalRepository.save("cafe", "goal_1", { x: 5, y: 6, theta: 1 });

    await service.startGoal("cafe", "goal_1");
    expect(actionGateway.lastGoal).toMatchObject({
      pose: { pose: { position: { x: 5, y: 6, z: 0 } } },
    });
    expect(service.getStatus()).toBe(NavStatus.ACTIVE);

    await service.cancelGoal();
    expect(actionGateway.cancelledHandles).toHaveLength(1);
    expect(service.getStatus()).toBe(NavStatus.CANCELED);
  });
});
