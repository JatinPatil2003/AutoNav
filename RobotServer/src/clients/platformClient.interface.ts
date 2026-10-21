export interface RosterEntryDto {
  assignmentId: string;
  userId: string;
  email: string;
  name: string;
  avatarUrl?: string;
  pinHash: string | null;
  pinSet: boolean;
  pinUpdatedCount: number;
}

export interface IPlatformClient {
  checkUserExists(email: string): Promise<boolean>;
  fetchRoster(): Promise<RosterEntryDto[]>;
  requestPinSetupOtp(email: string): Promise<void>;
  verifyPinSetupOtp(email: string, otp: string): Promise<{ pinSetupToken: string }>;
  confirmPinSetup(pinSetupToken: string, pin: string): Promise<RosterEntryDto>;
  /** Rotates this robot's own ROBOT_LINK_SECRET, authenticated with the current one. Returns the new plaintext secret — shown once. */
  rotateLinkSecret(): Promise<{ linkSecret: string }>;
}
