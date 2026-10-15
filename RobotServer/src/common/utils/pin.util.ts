import bcrypt from "bcryptjs";

/**
 * The PIN is only ever hashed on PlatformServer (source of truth for
 * credentials — see AuthService.confirmPinSetup there); RobotServer only
 * ever compares a locally-cached hash, never generates one itself.
 */
export async function comparePin(pin: string, pinHash: string): Promise<boolean> {
  return bcrypt.compare(pin, pinHash);
}
