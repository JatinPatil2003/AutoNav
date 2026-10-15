import { Router } from "express";
import { validate } from "../../common/middleware/validate.middleware";
import { asyncHandler } from "../../common/utils/asyncHandler.util";
import { MappingController } from "./mapping.controller";
import { saveMapSchema, waypointSchema } from "./mapping.validation";

/** Mounted at /api/mapping — SLAM session lifecycle and the save-map workflow. Manual drive during mapping is a future milestone's concern, not duplicated here. */
export function createMappingRouter(mappingController: MappingController): Router {
  const router = Router();

  /**
   * @openapi
   * /api/mapping/start:
   *   post:
   *     summary: Launch SLAM (slam_toolbox) and clear any leftover temp waypoints
   *     tags: [Mapping]
   *     responses:
   *       200:
   *         description: SLAM launch started
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: started } } }
   */
  router.post("/start", asyncHandler(mappingController.start));

  /**
   * @openapi
   * /api/mapping/stop:
   *   post:
   *     summary: Stop the SLAM launch
   *     tags: [Mapping]
   *     responses:
   *       200:
   *         description: SLAM launch stopped
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: stopped } } }
   */
  router.post("/stop", asyncHandler(mappingController.stop));

  /**
   * @openapi
   * /api/mapping/current/map:
   *   get:
   *     summary: Get the latest occupancy grid received on /map while SLAM is running
   *     tags: [Mapping]
   *     responses:
   *       200:
   *         description: The current in-progress map, or null if none received yet
   *         content:
   *           application/json:
   *             schema: { type: object, nullable: true }
   */
  router.get("/current/map", asyncHandler(mappingController.getCurrentMap));

  /**
   * @openapi
   * /api/mapping/current/location:
   *   get:
   *     summary: Get the latest live pose while SLAM is running
   *     description: AMCL isn't running during SLAM, so this is null until the TF2 lookup gap noted in telemetry.broadcaster.ts is resolved — see RobotServer/README.md "Known gaps".
   *     tags: [Mapping]
   *     responses:
   *       200:
   *         description: The current location, or null
   *         content:
   *           application/json:
   *             schema:
   *               oneOf:
   *                 - $ref: '#/components/schemas/Pose'
   *                 - type: object
   *                   nullable: true
   */
  router.get("/current/location", asyncHandler(mappingController.getCurrentLocation));

  /**
   * @openapi
   * /api/mapping/waypoints:
   *   post:
   *     summary: Capture the robot's current live pose as a named waypoint under the temp map
   *     tags: [Mapping]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               name: { type: string, example: doorway }
   *             required: [name]
   *     responses:
   *       201:
   *         description: Waypoint captured
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: captured } } }
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       503: { $ref: '#/components/responses/RosUnavailable' }
   */
  router.post("/waypoints", validate({ body: waypointSchema }), asyncHandler(mappingController.captureWaypoint));

  /**
   * @openapi
   * /api/mapping/waypoints/commit:
   *   post:
   *     summary: Move all temp waypoints onto a named map, then clear the temp map
   *     tags: [Mapping]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               mapName: { type: string, example: cafe }
   *             required: [mapName]
   *     responses:
   *       200:
   *         description: Waypoints committed
   *         content:
   *           application/json:
   *             schema:
   *               type: object
   *               properties:
   *                 status: { type: string, example: committed }
   *                 processed: { type: integer, description: "Number of waypoints moved", example: 3 }
   *       400: { $ref: '#/components/responses/ValidationError' }
   */
  router.post("/waypoints/commit", validate({ body: saveMapSchema }), asyncHandler(mappingController.commitWaypoints));

  /**
   * @openapi
   * /api/mapping/save:
   *   post:
   *     summary: Save the SLAM map to disk via map_saver_cli
   *     tags: [Mapping]
   *     requestBody:
   *       required: true
   *       content:
   *         application/json:
   *           schema:
   *             type: object
   *             properties:
   *               mapName: { type: string, example: cafe }
   *             required: [mapName]
   *     responses:
   *       200:
   *         description: Map saved
   *         content:
   *           application/json:
   *             schema: { type: object, properties: { status: { type: string, example: saved } } }
   *       400: { $ref: '#/components/responses/ValidationError' }
   *       503: { $ref: '#/components/responses/RosUnavailable' }
   */
  router.post("/save", validate({ body: saveMapSchema }), asyncHandler(mappingController.saveMap));

  return router;
}
