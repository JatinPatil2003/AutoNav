import fs from "node:fs";
import path from "node:path";
import { logger } from "../common/utils/logger.util";
import { config } from "./env";

const OVERRIDE_FILE = path.join(config.runtimeDir, "robot-link-secret.json");

interface PersistedSecret {
  linkSecret: string;
}

/**
 * Mutable holder for ROBOT_LINK_SECRET. `config` (env.ts) is a frozen
 * snapshot of env vars taken once at boot — fine for everything else, but
 * the maintenance module's credential-rotation action needs to change this
 * value while the process keeps running, and have it survive a restart
 * without regressing to the stale value baked into docker-compose.yml's
 * env. This prefers a persisted override file (on the RUNTIME_DIR volume)
 * over the env var if one exists, and `rotate()` updates both the in-memory
 * value and that file.
 *
 * A singleton module-level instance, matching how `config` itself is
 * consumed directly (`import { config } from "./env"`) rather than
 * dependency-injected — this is the one piece of config this codebase
 * already treats as a global, not a DI boundary.
 */
class LinkSecretStore {
  private current: string;

  constructor(envSecret: string) {
    this.current = this.loadPersisted() ?? envSecret;
  }

  getSecret(): string {
    return this.current;
  }

  /** Updates the live value immediately and persists it so a restart doesn't regress to the old one. Callers that need the robot-link connection to actually use this right away must force a reconnect themselves (see maintenance.service.ts). */
  rotate(newSecret: string): void {
    this.current = newSecret;
    this.persist(newSecret);
  }

  private loadPersisted(): string | null {
    try {
      const raw = fs.readFileSync(OVERRIDE_FILE, "utf-8");
      const parsed = JSON.parse(raw) as PersistedSecret;
      return parsed.linkSecret || null;
    } catch {
      // No override file yet (first boot after provisioning) or it's
      // unreadable/corrupt — fall back to the env var, same as always.
      return null;
    }
  }

  private persist(secret: string): void {
    try {
      fs.mkdirSync(path.dirname(OVERRIDE_FILE), { recursive: true });
      const payload: PersistedSecret = { linkSecret: secret };
      fs.writeFileSync(OVERRIDE_FILE, JSON.stringify(payload), { mode: 0o600 });
    } catch (err) {
      logger.error(
        { err },
        "linkSecretStore: failed to persist rotated secret to disk — it will revert to the previous value on restart",
      );
    }
  }
}

export const linkSecretStore = new LinkSecretStore(config.robotLinkSecret);
