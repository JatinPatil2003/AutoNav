import { NextFunction, Request, Response } from "express";
import { ForbiddenError } from "../errors";

export function requireAdmin(req: Request, _res: Response, next: NextFunction): void {
  if (req.user?.role !== "admin") {
    throw new ForbiddenError("Admin privileges required");
  }
  next();
}
