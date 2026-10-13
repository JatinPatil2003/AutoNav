export interface IEmailOtpSender {
  sendOtp(email: string, otp: string): Promise<void>;
}
