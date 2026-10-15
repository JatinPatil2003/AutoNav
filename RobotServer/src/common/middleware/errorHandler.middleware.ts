import { NextFunction, Request, Response } from "express";
import { ZodError } from "zod";
import { AppError } from "../errors";
import { logger } from "../utils/logger.util";

// eslint-disable-next-line @typescript-eslint/no-unused-vars
export function errorHandler(err: unknown, req: Request, res: Response, _next: NextFunction): void {
  if (err instanceof ZodError) {
    res.status(400).json({
      error: "ValidationError",
      message: "Request validation failed",
      details: err.issues.map((issue) => ({
        path: issue.path.join("."),
        message: issue.message,
      })),
    });
    return;
  }

  if (err instanceof AppError) {
    if (!err.isOperational) {
      logger.error({ err }, "Non-operational application error");
    }
    res.status(err.statusCode).json({
      error: err.constructor.name,
      message: err.message,
    });
    return;
  }

  logger.error({ err, path: req.path, method: req.method }, "Unhandled error");
  res.status(500).json({
    error: "InternalServerError",
    message: "An unexpected error occurred",
  });
}
