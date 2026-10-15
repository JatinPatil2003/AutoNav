import { Request, Response } from "express";

export function notFoundHandler(req: Request, res: Response): void {
  res.status(404).json({
    error: "NotFound",
    message: `Route ${req.method} ${req.originalUrl} not found`,
  });
}
