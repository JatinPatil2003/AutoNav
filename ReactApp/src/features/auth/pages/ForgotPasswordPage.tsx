import { useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { Button } from "../../../components/ui/Button";
import { Card } from "../../../components/ui/Card";
import { Input } from "../../../components/ui/Input";
import { PasswordInput } from "../../../components/ui/PasswordInput";
import { ThemeToggle } from "../../../components/ui/ThemeToggle";
import { getApiErrorMessage } from "../../../app/apiError";
import {
  useForgotPasswordMutation,
  useResetPasswordMutation,
  useVerifyOtpMutation,
} from "../authApi";
import {
  ForgotPasswordFormValues,
  ResetPasswordFormValues,
  VerifyOtpFormValues,
  forgotPasswordFormSchema,
  resetPasswordFormSchema,
  verifyOtpFormSchema,
} from "../auth.schemas";

type Step = "email" | "otp" | "reset" | "done";

/**
 * Three-step wizard: request OTP by email -> verify OTP for a resetToken ->
 * set a new password with that token. Each step is its own tiny form so
 * errors from one step don't leak into the next.
 */
export function ForgotPasswordPage() {
  const [step, setStep] = useState<Step>("email");
  const [email, setEmail] = useState("");
  const [resetToken, setResetToken] = useState("");
  const navigate = useNavigate();

  const [forgotPassword, { isLoading: isSendingOtp, error: forgotError }] =
    useForgotPasswordMutation();
  const [verifyOtp, { isLoading: isVerifying, error: verifyError }] = useVerifyOtpMutation();
  const [resetPassword, { isLoading: isResetting, error: resetError }] =
    useResetPasswordMutation();

  const emailForm = useForm<ForgotPasswordFormValues>({
    resolver: zodResolver(forgotPasswordFormSchema),
  });
  const otpForm = useForm<VerifyOtpFormValues>({ resolver: zodResolver(verifyOtpFormSchema) });
  const resetForm = useForm<ResetPasswordFormValues>({
    resolver: zodResolver(resetPasswordFormSchema),
  });

  async function onSubmitEmail(values: ForgotPasswordFormValues) {
    try {
      await forgotPassword(values).unwrap();
      setEmail(values.email);
      setStep("otp");
    } catch {
      // error surfaced below via forgotError
    }
  }

  async function onSubmitOtp(values: VerifyOtpFormValues) {
    try {
      const { resetToken: token } = await verifyOtp({ email, otp: values.otp }).unwrap();
      setResetToken(token);
      setStep("reset");
    } catch {
      // error surfaced below via verifyError
    }
  }

  async function onSubmitReset(values: ResetPasswordFormValues) {
    try {
      await resetPassword({ resetToken, newPassword: values.newPassword }).unwrap();
      setStep("done");
    } catch {
      // error surfaced below via resetError
    }
  }

  return (
    <div className="relative flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <div className="absolute right-4 top-4">
        <ThemeToggle />
      </div>
      <Card className="w-full max-w-sm">
        {step === "email" && (
          <>
            <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
              Forgot password
            </h1>
            <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
              Enter your account email and we&apos;ll send you a 6-digit reset code.
            </p>
            <form onSubmit={emailForm.handleSubmit(onSubmitEmail)} className="flex flex-col gap-4">
              <Input
                label="Email"
                type="email"
                autoComplete="email"
                error={emailForm.formState.errors.email?.message}
                {...emailForm.register("email")}
              />
              {forgotError && (
                <p className="text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(forgotError)}</p>
              )}
              <Button type="submit" isLoading={isSendingOtp} className="mt-2">
                Send reset code
              </Button>
            </form>
          </>
        )}

        {step === "otp" && (
          <>
            <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
              Enter reset code
            </h1>
            <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
              If an account exists for {email}, a 6-digit code was emailed to it. It expires in a
              few minutes.
            </p>
            <form onSubmit={otpForm.handleSubmit(onSubmitOtp)} className="flex flex-col gap-4">
              <Input
                label="6-digit code"
                inputMode="numeric"
                maxLength={6}
                error={otpForm.formState.errors.otp?.message}
                {...otpForm.register("otp")}
              />
              {verifyError && (
                <p className="text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(verifyError)}</p>
              )}
              <Button type="submit" isLoading={isVerifying} className="mt-2">
                Verify code
              </Button>
            </form>
          </>
        )}

        {step === "reset" && (
          <>
            <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
              Set a new password
            </h1>
            <form onSubmit={resetForm.handleSubmit(onSubmitReset)} className="mt-4 flex flex-col gap-4">
              <PasswordInput
                label="New password"
                autoComplete="new-password"
                error={resetForm.formState.errors.newPassword?.message}
                {...resetForm.register("newPassword")}
              />
              <PasswordInput
                label="Confirm password"
                autoComplete="new-password"
                error={resetForm.formState.errors.confirmPassword?.message}
                {...resetForm.register("confirmPassword")}
              />
              {resetError && (
                <p className="text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(resetError)}</p>
              )}
              <Button type="submit" isLoading={isResetting} className="mt-2">
                Reset password
              </Button>
            </form>
          </>
        )}

        {step === "done" && (
          <>
            <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
              Password reset
            </h1>
            <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
              Your password has been changed and you&apos;ve been logged out everywhere. Sign in
              with your new password.
            </p>
            <Button className="w-full" onClick={() => navigate("/login")}>
              Go to login
            </Button>
          </>
        )}

        {step !== "done" && (
          <p className="mt-6 text-center text-sm text-slate-500 dark:text-slate-400">
            <Link
              to="/login"
              className="font-medium text-brand-600 hover:text-brand-700 dark:text-brand-400"
            >
              Back to login
            </Link>
          </p>
        )}
      </Card>
    </div>
  );
}
