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
}
