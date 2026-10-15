import { AppError } from "./AppError";

/**
 * Marks a route that exists in the API contract but has no real
 * implementation behind it yet (see modules/system/*). Returns 501 instead
 * of silently answering `{ success: true }` for something that didn't
 * happen — that's the bug this replaces (see RobotServer/README.md).
 */
export class NotImplementedError extends AppError {
  constructor(message = "Not implemented yet") {
    super(message, 501);
  }
}
