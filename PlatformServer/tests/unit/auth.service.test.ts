import { AuthService } from "../../src/modules/auth/auth.service";
import { ConflictError, UnauthorizedError } from "../../src/common/errors";
import { FakeUserRepository } from "./fakes/FakeUserRepository";
import { FakePasswordResetOtpRepository } from "./fakes/FakePasswordResetOtpRepository";
import { FakeEmailOtpSender } from "./fakes/FakeEmailOtpSender";

describe("AuthService", () => {
  let authService: AuthService;
  let userRepository: FakeUserRepository;
  let otpRepository: FakePasswordResetOtpRepository;
  let emailOtpSender: FakeEmailOtpSender;

  beforeEach(() => {
    userRepository = new FakeUserRepository();
    otpRepository = new FakePasswordResetOtpRepository();
    emailOtpSender = new FakeEmailOtpSender();
    authService = new AuthService(userRepository, otpRepository, emailOtpSender);
  });

  it("creates a new local account on signup", async () => {
    const { user, tokens } = await authService.signup({
      email: "jane@example.com",
      password: "password123",
      name: "Jane Doe",
    });

    expect(user.email).toBe("jane@example.com");
    expect(user.role).toBe("user");
    expect(user.robot).toBeNull();
    expect(tokens.accessToken).toEqual(expect.any(String));
    expect(tokens.refreshToken).toEqual(expect.any(String));
  });

  it("grants the admin role to emails listed in ADMIN_EMAILS", async () => {
    const { user } = await authService.signup({
      email: "admin@example.com",
      password: "password123",
      name: "Admin User",
    });

    expect(user.role).toBe("admin");
  });

  it("rejects signup with a duplicate email", async () => {
    await authService.signup({ email: "dup@example.com", password: "password123", name: "A" });

    await expect(
      authService.signup({ email: "dup@example.com", password: "password123", name: "B" }),
    ).rejects.toBeInstanceOf(ConflictError);
  });

  it("logs in with correct credentials", async () => {
    await authService.signup({ email: "login@example.com", password: "correct-pw", name: "Login" });

    const { user } = await authService.login({ email: "login@example.com", password: "correct-pw" });

    expect(user.email).toBe("login@example.com");
  });

  it("rejects login with an incorrect password", async () => {
    await authService.signup({ email: "login2@example.com", password: "correct-pw", name: "Login2" });

    await expect(
      authService.login({ email: "login2@example.com", password: "wrong-pw" }),
    ).rejects.toBeInstanceOf(UnauthorizedError);
  });

  it("rejects login for an unknown email", async () => {
    await expect(
      authService.login({ email: "nobody@example.com", password: "whatever" }),
    ).rejects.toBeInstanceOf(UnauthorizedError);
  });

  it("issues a new token pair on refresh and revokes it on logout", async () => {
    const { user, tokens } = await authService.signup({
      email: "refresh@example.com",
      password: "password123",
      name: "Refresh",
    });

    const refreshed = await authService.refresh(tokens.refreshToken);
    expect(refreshed.accessToken).toEqual(expect.any(String));

    await authService.logout(user.id);

    await expect(authService.refresh(tokens.refreshToken)).rejects.toBeInstanceOf(UnauthorizedError);
  });

  it("completes the forgot-password OTP flow end to end", async () => {
    await authService.signup({
      email: "reset@example.com",
      password: "old-password",
      name: "Reset Me",
    });

    await authService.forgotPassword({ email: "reset@example.com" });
    expect(emailOtpSender.sent).toHaveLength(1);
    const otp = emailOtpSender.sent[0].otp;

    const { resetToken } = await authService.verifyOtp({ email: "reset@example.com", otp });
    await authService.resetPassword({ resetToken, newPassword: "new-password" });

    await expect(
      authService.login({ email: "reset@example.com", password: "old-password" }),
    ).rejects.toBeInstanceOf(UnauthorizedError);

    const { user } = await authService.login({
      email: "reset@example.com",
      password: "new-password",
    });
    expect(user.email).toBe("reset@example.com");
  });

  it("rejects an incorrect OTP", async () => {
    await authService.signup({ email: "badotp@example.com", password: "pw", name: "Bad Otp" });
    await authService.forgotPassword({ email: "badotp@example.com" });

    await expect(
      authService.verifyOtp({ email: "badotp@example.com", otp: "000000" }),
    ).rejects.toBeInstanceOf(UnauthorizedError);
  });

  it("does not error when forgot-password is called for an unknown email", async () => {
    await expect(
      authService.forgotPassword({ email: "nobody@example.com" }),
    ).resolves.toBeUndefined();
    expect(emailOtpSender.sent).toHaveLength(0);
  });
});
