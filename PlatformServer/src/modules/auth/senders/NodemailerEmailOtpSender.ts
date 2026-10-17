import nodemailer, { Transporter } from "nodemailer";
import { config } from "../../../config/env";
import { logger } from "../../../common/utils/logger.util";
import { IEmailOtpSender } from "../otpSender.interface";

/**
 * Free email delivery via Gmail SMTP + an app password — no third-party
 * signup, no cost. Swap for an SES/Resend implementation of the same
 * interface later without touching AuthService.
 */
export class NodemailerEmailOtpSender implements IEmailOtpSender {
  private transporter: Transporter;

  constructor() {
    this.transporter = nodemailer.createTransport({
      service: "gmail",
      pool: true,
      maxConnections: 3,
      maxMessages: 100,
      auth: { user: config.email.user, pass: config.email.appPassword },
    });
  }

  async sendOtp(email: string, otp: string): Promise<void> {
    await this.transporter.sendMail({
      from: `"AutoNav" <${config.email.user}>`,
      to: email,
      subject: "Your AutoNav password reset code",
      text: `Your password reset code is ${otp}. It expires in ${config.otp.expiryMinutes} minutes. If you didn't request this, ignore this email.`,
    });
    logger.debug({ email }, "NodemailerEmailOtpSender: OTP email sent");
  }
}
