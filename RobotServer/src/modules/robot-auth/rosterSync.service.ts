import { IPlatformClient } from "../../clients/platformClient.interface";
import { DeviceIdentityService } from "../../ros/deviceIdentity";
import { ILocalUserRepository } from "./localUser.repository.interface";
import { logger } from "../../common/utils/logger.util";

/**
 * Pulls the current roster from PlatformServer and replaces the local
 * cache wholesale. Called on boot and on a periodic interval (see index.ts)
 * as a staleness upper bound, and additionally on-demand whenever
 * PlatformServer pushes a roster-invalidate signal (see
 * platformAuth.routes.ts) after an admin grants/revokes access.
 */
export class RosterSyncService {
  constructor(
    private readonly platformClient: IPlatformClient,
    private readonly localUserRepository: ILocalUserRepository,
    private readonly deviceIdentity: DeviceIdentityService,
  ) {}

  async sync(): Promise<void> {
    const robotId = await this.deviceIdentity.resolve();
    if (!robotId) {
      logger.warn("rosterSync: device id not resolved yet, skipping sync");
      return;
    }

    const entries = await this.platformClient.fetchRoster();
    await this.localUserRepository.replaceAll(entries, robotId);
    logger.info({ count: entries.length, robotId }, "rosterSync: local roster cache refreshed");
  }
}
