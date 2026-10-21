/**
 * Manual smoke test for the five rclnodejs call shapes RobotServer's
 * README flags as "never verified against a live ROS2 graph" — run by
 * hand against the real robot (needs a sourced ROS environment and
 * whatever ROS2 nodes are relevant to the check already launched), never
 * in CI. Exercises each shape in isolation, reports pass/fail/skip for
 * each, and never throws past a single check — one failing shape
 * shouldn't stop the rest from being tried.
 *
 * Usage (from RobotServer/, with the ROS env sourced):
 *   npm run smoke:ros
 *
 * The NavigateToPose check only confirms the action SERVER is reachable —
 * it deliberately does not send a real goal, since that would move the
 * robot as a side effect of running a diagnostic script.
 */
import * as rclnodejs from "rclnodejs";
import { initRosNode, shutdownRosNode } from "../src/ros/rosNode";
import { RosServiceGateway } from "../src/ros/rosServiceGateway";
import { RosTopicGateway } from "../src/ros/rosTopicGateway";

type CheckResult = "PASS" | "FAIL" | "SKIP";

interface CheckReport {
  name: string;
  result: CheckResult;
  detail: string;
}

const TOPIC_WAIT_MS = 5000;

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function checkGetDeviceId(serviceGateway: RosServiceGateway): Promise<CheckReport> {
  const name = "std_srvs/srv/Trigger on /esp32_bridge_node/get_device_id";
  try {
    const response = await serviceGateway.call<Record<string, never>, { success: boolean; message: string }>(
      "std_srvs/srv/Trigger",
      "/esp32_bridge_node/get_device_id",
      {},
      5000,
    );
    if (response.success && response.message) {
      return { name, result: "PASS", detail: `device id = "${response.message}"` };
    }
    return { name, result: "FAIL", detail: `responded but success=${response.success} message="${response.message}"` };
  } catch (err) {
    return { name, result: "FAIL", detail: String(err) };
  }
}

async function checkSetLedPattern(serviceGateway: RosServiceGateway): Promise<CheckReport> {
  const name = "autonav_msgs/srv/SetLedPattern on /esp32_bridge_node/led/set_pattern";
  try {
    const response = await serviceGateway.call<Record<string, unknown>, { success: boolean; message: string }>(
      "autonav_msgs/srv/SetLedPattern",
      "/esp32_bridge_node/led/set_pattern",
      { pattern_id: 0, use_custom_params: false, hue: 0, sat: 0, val: 0, speed_ms: 0 },
      5000,
    );
    return response.success
      ? { name, result: "PASS", detail: response.message }
      : { name, result: "FAIL", detail: `success=false message="${response.message}"` };
  } catch (err) {
    return { name, result: "FAIL", detail: String(err) };
  }
}

async function checkSetInitialPose(serviceGateway: RosServiceGateway): Promise<CheckReport> {
  const name = "nav2_msgs/srv/SetInitialPose on /set_initial_pose";
  try {
    await serviceGateway.call(
      "nav2_msgs/srv/SetInitialPose",
      "/set_initial_pose",
      {
        pose: {
          header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
          pose: {
            pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
            covariance: new Array(36).fill(0),
          },
        },
      },
      3000,
    );
    return { name, result: "PASS", detail: "service accepted the request (requires AMCL/Nav2 already running)" };
  } catch (err) {
    return { name, result: "FAIL", detail: `${String(err)} (expected if Nav2 isn't currently running)` };
  }
}

async function checkNavigateToPoseServerReachable(node: rclnodejs.Node): Promise<CheckReport> {
  const name = "nav2_msgs/action/NavigateToPose server reachability (no goal sent)";
  try {
    // RosActionGateway.sendGoal() always sends a real goal once the server
    // answers waitForServer() — deliberately not used here, since that
    // would move the robot. We only need rclnodejs's ActionClient
    // construction + waitForServer() to prove the type/action-name shape
    // resolves, which is the actual thing under test.
    const client = new rclnodejs.ActionClient(
      node,
      "nav2_msgs/action/NavigateToPose" as unknown as rclnodejs.TypeClass<rclnodejs.ActionTypeClassName>,
      "navigate_to_pose",
    );
    const available = await client.waitForServer(3000);
    return available
      ? { name, result: "PASS", detail: "action server reachable" }
      : { name, result: "FAIL", detail: "action server not reachable (expected if Nav2 isn't currently running)" };
  } catch (err) {
    return { name, result: "FAIL", detail: String(err) };
  }
}

async function checkTopicSubscription(topicGateway: RosTopicGateway, type: string, topicName: string): Promise<CheckReport> {
  const name = `${type} subscription on ${topicName}`;
  topicGateway.subscribe(type, topicName);
  await sleep(TOPIC_WAIT_MS);
  const latest = topicGateway.getLatest(topicName);
  if (latest) {
    return { name, result: "PASS", detail: "received at least one message" };
  }
  return {
    name,
    result: "FAIL",
    detail:
      `no message received in ${TOPIC_WAIT_MS}ms. If the topic already has a publisher and a previously-published ` +
      `message (e.g. a transient-local map), this likely means RosTopicGateway's subscription QoS doesn't match the ` +
      `publisher's — cross-check with "ros2 topic info ${topicName} --verbose".`,
  };
}

function printReport(reports: CheckReport[]): void {
  console.log("\n=== ROS smoke test results ===\n");
  for (const report of reports) {
    const marker = report.result === "PASS" ? "✅" : report.result === "SKIP" ? "⏭️ " : "❌";
    console.log(`${marker} [${report.result}] ${report.name}`);
    console.log(`    ${report.detail}`);
  }
  const failed = reports.filter((r) => r.result === "FAIL").length;
  console.log(`\n${reports.length - failed}/${reports.length} checks passed.\n`);
}

async function main(): Promise<void> {
  const node = await initRosNode("robot_server_smoke_test");
  const serviceGateway = new RosServiceGateway(node);
  const topicGateway = new RosTopicGateway(node);

  const reports: CheckReport[] = [];
  reports.push(await checkGetDeviceId(serviceGateway));
  reports.push(await checkSetLedPattern(serviceGateway));
  reports.push(await checkSetInitialPose(serviceGateway));
  reports.push(await checkNavigateToPoseServerReachable(node));
  reports.push(await checkTopicSubscription(topicGateway, "nav_msgs/msg/OccupancyGrid", "/map"));
  reports.push(await checkTopicSubscription(topicGateway, "geometry_msgs/msg/PoseWithCovarianceStamped", "/amcl_pose"));

  printReport(reports);
  await shutdownRosNode();
  process.exit(reports.some((r) => r.result === "FAIL") ? 1 : 0);
}

main().catch((err) => {
  console.error("ros-smoke-test crashed:", err);
  process.exit(1);
});
