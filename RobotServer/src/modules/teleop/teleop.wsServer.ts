import { z } from "zod";
import { WsRouter } from "../../core/wsRouter";
import { logger } from "../../common/utils/logger.util";
import { TeleopService } from "./teleop.service";
import { velocitySchema } from "./teleop.validation";

const joystickMessageSchema = z.object({
  type: z.literal("joystick"),
  data: velocitySchema,
});

/** Registers /ws/teleop — lower-latency joystick input than a REST call per tick. */
export function registerTeleopWebSocket(wsRouter: WsRouter, teleopService: TeleopService): void {
  wsRouter.register("/ws/teleop", (socket) => {
    socket.on("message", (raw) => {
      let parsed: unknown;
      try {
        parsed = JSON.parse(raw.toString());
      } catch {
        return;
      }

      const message = joystickMessageSchema.safeParse(parsed);
      if (!message.success) {
        logger.debug({ issues: message.error.issues }, "ignoring malformed teleop websocket message");
        return;
      }

      teleopService.setVelocity(message.data.data);
    });
  });
}
