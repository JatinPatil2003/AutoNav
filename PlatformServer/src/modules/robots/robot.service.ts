import { ConflictError } from "../../common/errors";
import { IRobotRepository } from "./robot.repository.interface";
import { RobotStatus } from "./robot.model";
import { RobotDto, toRobotDto } from "./robot.dto";
import { CreateRobotDto } from "./robot.validation";

export class RobotService {
  constructor(private readonly robotRepository: IRobotRepository) {}

  async createRobot(input: CreateRobotDto): Promise<RobotDto> {
    const existing = await this.robotRepository.findByName(input.name);
    if (existing) {
      throw new ConflictError(`Robot with name "${input.name}" already exists`);
    }

    const robot = await this.robotRepository.create(input);
    return toRobotDto(robot);
  }

  async listByStatus(status: RobotStatus): Promise<RobotDto[]> {
    const robots = await this.robotRepository.listByStatus(status);
    return robots.map(toRobotDto);
  }

  async listAll(): Promise<RobotDto[]> {
    const robots = await this.robotRepository.listAll();
    return robots.map(toRobotDto);
  }
}
