import { NotFoundError } from "../../common/errors";
import { IUserRepository } from "./user.repository.interface";
import { PublicUserDto, toPublicUserDto } from "./user.dto";

export class UserService {
  constructor(private readonly userRepository: IUserRepository) {}

  async getMe(userId: string): Promise<PublicUserDto> {
    const user = await this.userRepository.findByIdPopulated(userId);

    if (!user) {
      throw new NotFoundError("User not found");
    }

    return toPublicUserDto(user);
  }
}
