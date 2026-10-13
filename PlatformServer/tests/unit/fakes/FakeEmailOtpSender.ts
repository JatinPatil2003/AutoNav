import { IEmailOtpSender } from "../../../src/modules/auth/otpSender.interface";

/** Captures sent OTPs in-memory instead of hitting real SMTP in tests. */
export class FakeEmailOtpSender implements IEmailOtpSender {
  public sent: { email: string; otp: string }[] = [];

  async sendOtp(email: string, otp: string): Promise<void> {
    this.sent.push({ email, otp });
  }
}
