import { RosterEntryDto } from "../../clients/platformClient.interface";
import { ILocalUserDocument } from "./localUser.model";

export interface ILocalUserRepository {
  findByEmail(email: string): Promise<ILocalUserDocument | null>;
  /** The roster shown on the lock screen — active users only. */
  listActive(): Promise<ILocalUserDocument[]>;
  /** Full replace: upserts every entry, and marks any local user not present in `entries` as revoked. */
  replaceAll(entries: RosterEntryDto[]): Promise<void>;
  /** Single-entry upsert, used right after a pin-setup confirm to avoid waiting for the next full sync. */
  applyEntry(entry: RosterEntryDto): Promise<ILocalUserDocument>;
  recordFailedAttempt(id: string, maxAttempts: number, lockoutMinutes: number): Promise<ILocalUserDocument | null>;
  resetAttempts(id: string): Promise<void>;
}
