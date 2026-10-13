import { Request, Response } from "express";
import { UnauthorizedError } from "../../common/errors";
import { UserService } from "./user.service";

export class UserController {
  constructor(private readonly userService: UserService) {}

  getMe = async (req: Request, res: Response): Promise<void> => {
    if (!req.user) {
      throw new UnauthorizedError();
    }

    const user = await this.userService.getMe(req.user.id);
    res.status(200).json({ user });
  };
}
