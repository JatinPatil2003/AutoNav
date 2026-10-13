import swaggerJsdoc from "swagger-jsdoc";

export const swaggerSpec = swaggerJsdoc({
  definition: {
    openapi: "3.0.3",
    info: {
      title: "AutoNav Platform API",
      version: "0.1.0",
      description:
        "Auth, user, and robot-assignment API for the AutoNav operator dashboard. " +
        "Served exclusively over HTTPS in production (see AWS/nginx/dashboard.autonav.in.conf).",
    },
    servers: [{ url: "/" }],
    components: {
      securitySchemes: {
        cookieAuth: {
          type: "apiKey",
          in: "cookie",
          name: "accessToken",
        },
      },
    },
  },
  apis: ["./src/modules/**/*.routes.ts"],
});
