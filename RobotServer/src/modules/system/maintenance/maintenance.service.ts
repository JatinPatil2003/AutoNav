import fs from "node:fs/promises";
import path from "node:path";
import { IPlatformClient } from "../../../clients/platformClient.interface";
import { IPlatformLink } from "../../../clients/platformLink.interface";
import { logger } from "../../../common/utils/logger.util";
import { linkSecretStore } from "../../../config/linkSecretStore";

/** Watched by deploy/maintenance/autonav-reboot.{path,service} on the host — see that unit's comments. */
export const REBOOT_TRIGGER_FILE = "reboot-requested";

/** The one piece of linkSecretStore.ts's surface this service needs — a seam for tests, since the real module is a singleton touching the real filesystem (same reasoning as config.ts's own exemption from DI elsewhere in this codebase). */
export type LinkSecretStore = Pick<typeof linkSecretStore, "getSecret" | "rotate">;

export class MaintenanceService {
  private link: IPlatformLink | undefined;

  constructor(
    private readonly platformClient: IPlatformClient,
    private readonly runtimeDir: string,
    private readonly secretStore: LinkSecretStore = linkSecretStore,
  ) {}

  /**
   * Wired in once app.ts's registerPlatformLink callback has the real
   * IPlatformLink instance. That callback only fires after index.ts
   * constructs PlatformLinkClient — strictly later than createApp() builds
   * this service — so this can't be a constructor dependency the way most
   * of this codebase's services take theirs; see telemetry.broadcaster.ts's
   * registration for the same constraint on the same instance.
   */
  setPlatformLink(link: IPlatformLink): void {
    this.link = link;
  }

  /**
   * RobotServer's own container has no privilege to reboot the host
   * (no `privileged: true`, no D-Bus/systemd access — confirmed absent
   * when this was designed). Rather than adding that privilege, this just
   * writes a trigger file onto the RUNTIME_DIR volume; a host-level
   * systemd path unit (deploy/maintenance/autonav-reboot.path/.service,
   * following the same host-systemd-unit precedent as
   * deploy/kiosk/robotapp-kiosk.service) watches for it and performs the
   * actual `systemctl reboot` outside Docker entirely once installed.
   */
  async requestReboot(): Promise<void> {
    await fs.mkdir(this.runtimeDir, { recursive: true });
    await fs.writeFile(path.join(this.runtimeDir, REBOOT_TRIGGER_FILE), new Date().toISOString());
    logger.warn("maintenance: reboot requested — trigger file written, the host companion unit should reboot shortly");
  }

  /**
   * Rotates ROBOT_LINK_SECRET end to end: asks PlatformServer for a new one
   * (authenticated with the current secret — see
   * resolveRobot.middleware.ts/robotAuth.routes.ts's rotate-link-secret
   * route on that side), applies it to the live in-memory+persisted store,
   * then forces the robot-link connection to reconnect immediately rather
   * than keep using the now-dead old secret until its next natural
   * disconnect.
   */
  async rotateCredentials(): Promise<{ linkSecret: string }> {
    const { linkSecret } = await this.platformClient.rotateLinkSecret();
    this.secretStore.rotate(linkSecret);
    this.link?.forceReconnect();
    return { linkSecret };
  }
}
