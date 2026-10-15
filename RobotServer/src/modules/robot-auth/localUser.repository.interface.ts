import { RosterEntryDto } from "../../clients/platformClient.interface";
import { ILocalUserDocument } from "./localUser.model";

export interface ILocalUserRepository {
  /** Scoped to this robot's own id — a cached row from a different robot id can never be matched. */
  findByEmail(email: string, robotId: string): Promise<ILocalUserDocument | null>;
  /** The roster shown on the lock screen — active users on this robot only. */
  listActive(robotId: string): Promise<ILocalUserDocument[]>;
  /** Full replace: upserts every entry (stamped with robotId), and marks any local user on this robot not present in `entries` as revoked. */
  replaceAll(entries: RosterEntryDto[], robotId: string): Promise<void>;
  /** Single-entry upsert, used right after a pin-setup confirm to avoid waiting for the next full sync. */
  applyEntry(entry: RosterEntryDto, robotId: string): Promise<ILocalUserDocument>;
  recordFailedAttempt(id: string, maxAttempts: number, lockoutMinutes: number): Promise<ILocalUserDocument | null>;
  resetAttempts(id: string): Promise<void>;
}
