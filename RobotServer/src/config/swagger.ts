import swaggerJsdoc from "swagger-jsdoc";

/**
 * Static schema/response components shared across every route's `@openapi`
 * JSDoc block (see src/modules/**\/*.routes.ts), so the common error shapes
 * (src/common/errors, errorHandler.middleware.ts) are described once and
 * referenced via $ref instead of repeated per-route.
 */
const components = {
  schemas: {
    ErrorResponse: {
      type: "object",
      properties: {
        error: { type: "string", description: "Error class name", example: "NotFoundError" },
        message: { type: "string", example: "Resource not found" },
      },
      required: ["error", "message"],
    },
    ValidationErrorResponse: {
      type: "object",
      properties: {
        error: { type: "string", example: "ValidationError" },
        message: { type: "string", example: "Request validation failed" },
        details: {
          type: "array",
          items: {
            type: "object",
            properties: {
              path: { type: "string", example: "linear" },
              message: { type: "string", example: "Expected number, received string" },
            },
          },
        },
      },
      required: ["error", "message", "details"],
    },
    Pose: {
      type: "object",
      properties: {
        x: { type: "number", example: 1.5 },
        y: { type: "number", example: -0.3 },
        theta: { type: "number", description: "Yaw, radians", example: 1.5708 },
      },
      required: ["x", "y", "theta"],
    },
  },
  responses: {
    ValidationError: {
      description: "Request body failed schema validation",
      content: { "application/json": { schema: { $ref: "#/components/schemas/ValidationErrorResponse" } } },
    },
    NotFound: {
      description: "The requested resource doesn't exist",
      content: { "application/json": { schema: { $ref: "#/components/schemas/ErrorResponse" } } },
    },
    RosUnavailable: {
      description: "The requested ROS component isn't available right now (e.g. Nav2 hasn't started)",
      content: { "application/json": { schema: { $ref: "#/components/schemas/ErrorResponse" } } },
    },
    NotImplemented: {
      description: "Defined in the API contract but not implemented yet (see RobotServer/README.md)",
      content: { "application/json": { schema: { $ref: "#/components/schemas/ErrorResponse" } } },
    },
  },
};

export const swaggerSpec = swaggerJsdoc({
  definition: {
    openapi: "3.0.3",
    info: {
      title: "AutoNav Robot API",
      version: "0.1.0",
      description:
        "Robot-side API bridging the ROS 2 graph to HTTP/WebSocket/WebRTC. " +
        "No auth of its own — the OpenVPN tunnel is the trust boundary (see root README).",
    },
    servers: [{ url: "/" }],
    tags: [
      { name: "Teleop", description: "Manual drive and emergency stop" },
      { name: "Navigation", description: "Nav2 goals, saved poses, localize-at-point" },
      { name: "Mapping", description: "SLAM session lifecycle and the save-map workflow" },
      { name: "Firmware", description: "ESP32 peripheral board: LED patterns and brightness" },
      { name: "System/Network", description: "WiFi + VPN control (not implemented yet)" },
      { name: "System/Diagnostics", description: "Sensor health and firmware/image version (not implemented yet)" },
      { name: "System/Maintenance", description: "Reboot, NTP sync, DB reset, credentials (not implemented yet)" },
    ],
    components,
  },
  apis: ["./src/modules/**/*.routes.ts"],
});
