import * as rclnodejs from "rclnodejs";
import { logger } from "../common/utils/logger.util";

/**
 * Owns the single rclnodejs Node for this process. Everything else in
 * ros/* takes the node as a constructor argument instead of reaching for a
 * module-level global — see modules/*.gateway.ts.
 */
let node: rclnodejs.Node | undefined;
let initialized = false;

export async function initRosNode(nodeName = "robot_server"): Promise<rclnodejs.Node> {
  if (node) {
    return node;
  }

  await rclnodejs.init();
  initialized = true;
  node = new rclnodejs.Node(nodeName);
  rclnodejs.spin(node);
  logger.info(`ROS node "${nodeName}" started`);
  return node;
}

export function getRosNode(): rclnodejs.Node {
  if (!node) {
    throw new Error("ROS node has not been initialized yet — call initRosNode() first");
  }
  return node;
}

export async function shutdownRosNode(): Promise<void> {
  if (node) {
    node.destroy();
    node = undefined;
  }
  if (initialized) {
    await rclnodejs.shutdown();
    initialized = false;
  }
  logger.info("ROS node shut down");
}
