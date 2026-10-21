export type SlamBackend = "cartographer" | "slam_toolbox";

export const DEFAULT_SLAM_BACKEND: SlamBackend = "cartographer";

/** autonav_navigation's launch file for each backend — both also launch autonav_utils's tf_pose_bridge node. */
export const SLAM_LAUNCH_FILE: Record<SlamBackend, string> = {
  cartographer: "cartographer.launch.py",
  slam_toolbox: "slam.launch.py",
};
