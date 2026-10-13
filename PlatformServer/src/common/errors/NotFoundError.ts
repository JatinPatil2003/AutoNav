import { AppError } from "./AppError";

export class NotFoundError extends AppError {
  constructor(message = "Not found") {
    super(message, 404);
  }
}
