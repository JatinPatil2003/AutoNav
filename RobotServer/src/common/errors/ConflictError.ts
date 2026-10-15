import { AppError } from "./AppError";

export class ConflictError extends AppError {
  constructor(message = "Conflicting state") {
    super(message, 409);
  }
}
