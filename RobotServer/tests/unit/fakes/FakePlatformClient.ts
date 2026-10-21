import bcrypt from "bcryptjs";
import { IPlatformClient, RosterEntryDto } from "../../../src/clients/platformClient.interface";

/**
 * Stands in for PlatformServer in tests — pre-seed `roster` and `otps`,
 * and the fake behaves like a real PlatformServer would for the pin-setup
 * relay (issuing a pinSetupToken on verify, applying the pin on confirm).
 */
export class FakePlatformClient implements IPlatformClient {
  public roster: RosterEntryDto[] = [];
  /** Emails considered to have a PlatformServer account but no access to this robot. */
  public accountsWithoutAccess = new Set<string>();
  private pendingOtp = new Map<string, string>();
  private pinSetupTokens = new Map<string, string>();

  async checkUserExists(email: string): Promise<boolean> {
    return this.roster.some((e) => e.email === email) || this.accountsWithoutAccess.has(email);
  }

  async fetchRoster(): Promise<RosterEntryDto[]> {
    return this.roster;
  }

  seedOtp(email: string, otp: string): void {
    this.pendingOtp.set(email, otp);
  }

  async requestPinSetupOtp(_email: string): Promise<void> {
    // No-op in the fake — tests seed the OTP directly via seedOtp().
  }

  async verifyPinSetupOtp(email: string, otp: string): Promise<{ pinSetupToken: string }> {
    if (this.pendingOtp.get(email) !== otp) {
      throw new Error("Invalid or expired code");
    }
    const pinSetupToken = `token-for-${email}`;
    this.pinSetupTokens.set(pinSetupToken, email);
    return { pinSetupToken };
  }

  async confirmPinSetup(pinSetupToken: string, pin: string): Promise<RosterEntryDto> {
    const email = this.pinSetupTokens.get(pinSetupToken);
    if (!email) throw new Error("Invalid pin setup token");

    const entry = this.roster.find((e) => e.email === email);
    if (!entry) throw new Error("No roster entry for this user");

    entry.pinHash = await bcrypt.hash(pin, 12);
    entry.pinSet = true;
    entry.pinUpdatedCount += 1;
    return entry;
  }

  rotatedSecret: string | null = null;

  async rotateLinkSecret(): Promise<{ linkSecret: string }> {
    this.rotatedSecret = "fake-rotated-secret";
    return { linkSecret: this.rotatedSecret };
  }
}
