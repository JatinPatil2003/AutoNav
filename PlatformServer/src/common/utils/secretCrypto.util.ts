import crypto from "crypto";
import { config } from "../../config/env";
import { IEncryptedSecret } from "../../modules/robots/robot.model";

const ALGORITHM = "aes-256-gcm";
const IV_LENGTH = 12;

/**
 * Robot secrets must be recoverable (not one-way hashed) because
 * PlatformServer itself needs the plaintext to sign outbound platform->robot
 * calls (see robot-auth module). AES-256-GCM with a server-wide master key
 * keeps the plaintext out of the database at rest while still allowing
 * decrypt-and-compare / decrypt-and-sign on this side.
 */
export function encryptSecret(plaintext: string): IEncryptedSecret {
  const key = Buffer.from(config.robotSecretEncKey, "base64");
  const iv = crypto.randomBytes(IV_LENGTH);
  const cipher = crypto.createCipheriv(ALGORITHM, key, iv);
  const ciphertext = Buffer.concat([cipher.update(plaintext, "utf8"), cipher.final()]);
  const authTag = cipher.getAuthTag();

  return {
    ciphertext: ciphertext.toString("base64"),
    iv: iv.toString("base64"),
    authTag: authTag.toString("base64"),
  };
}

export function decryptSecret(encrypted: IEncryptedSecret): string {
  const key = Buffer.from(config.robotSecretEncKey, "base64");
  const iv = Buffer.from(encrypted.iv, "base64");
  const decipher = crypto.createDecipheriv(ALGORITHM, key, iv);
  decipher.setAuthTag(Buffer.from(encrypted.authTag, "base64"));
  const plaintext = Buffer.concat([
    decipher.update(Buffer.from(encrypted.ciphertext, "base64")),
    decipher.final(),
  ]);
  return plaintext.toString("utf8");
}

export function generateRobotSecret(): string {
  return crypto.randomBytes(32).toString("base64url");
}

/**
 * A non-secret, indexable fingerprint of a robot secret — lets requireRobotAuth
 * find "which robot is this?" with a single indexed lookup instead of
 * decrypting and comparing every robot's stored secret. Safe to store
 * alongside the encrypted secret: SHA-256 of a 256-bit random value can't be
 * reversed to recover the secret, and collisions are practically impossible.
 */
export function hashSecretForLookup(secret: string): string {
  return crypto.createHash("sha256").update(secret, "utf8").digest("hex");
}

export function timingSafeEqual(a: string, b: string): boolean {
  const bufA = Buffer.from(a, "utf8");
  const bufB = Buffer.from(b, "utf8");
  if (bufA.length !== bufB.length) return false;
  return crypto.timingSafeEqual(bufA, bufB);
}
