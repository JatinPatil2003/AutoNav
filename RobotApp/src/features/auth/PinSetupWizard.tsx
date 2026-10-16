import { useEffect, useState } from "react";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { ArrowLeft, MailCheck, ShieldCheck } from "lucide-react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { Input } from "../../components/ui/Input";
import { AppShell } from "../../components/ui/AppShell";
import { UserAvatar } from "../../components/ui/UserAvatar";
import { getApiErrorMessage } from "../../api/apiError";
import { hmiClient } from "../../api/hmiClient";
import { maskEmail } from "../../utils/maskEmail";
import { OtpFormValues, SetPinFormValues, otpFormSchema, setPinFormSchema } from "./auth.schemas";

type Step = "otp" | "set-pin";

interface PinSetupWizardProps {
  email: string;
  name?: string;
  avatarUrl?: string;
  onBack: () => void;
  onPinSet: () => void;
}

/**
 * First-time-on-this-robot flow (also reused as "forgot PIN"): verify an
 * email OTP (sent/verified by PlatformServer, relayed through RobotServer —
 * see RobotServer/src/clients/platformClient.ts) then set a 4-digit PIN.
 * Mirrors PlatformApp's ForgotPasswordPage step-wizard shape.
 */
export function PinSetupWizard({ email, name, avatarUrl, onBack, onPinSet }: PinSetupWizardProps) {
  const [step, setStep] = useState<Step>("otp");
  const [pinSetupToken, setPinSetupToken] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const otpForm = useForm<OtpFormValues>({ resolver: zodResolver(otpFormSchema) });
  const pinForm = useForm<SetPinFormValues>({ resolver: zodResolver(setPinFormSchema) });

  async function requestOtp() {
    setError(null);
    try {
      await hmiClient.requestPinSetupOtp(email);
    } catch (err) {
      setError(getApiErrorMessage(err));
    }
  }

  useEffect(() => {
    requestOtp();
    // eslint-disable-next-line react-hooks/exhaustive-deps -- send the first code once, on entering this wizard
  }, []);

  async function onSubmitOtp(values: OtpFormValues) {
    setError(null);
    setIsSubmitting(true);
    try {
      const { pinSetupToken: token } = await hmiClient.verifyPinSetupOtp(email, values.otp);
      setPinSetupToken(token);
      setStep("set-pin");
    } catch (err) {
      setError(getApiErrorMessage(err));
    } finally {
      setIsSubmitting(false);
    }
  }

  async function onSubmitPin(values: SetPinFormValues) {
    setError(null);
    setIsSubmitting(true);
    try {
      await hmiClient.confirmPinSetup(pinSetupToken, values.pin);
      onPinSet();
    } catch (err) {
      setError(getApiErrorMessage(err));
    } finally {
      setIsSubmitting(false);
    }
  }

  return (
    <AppShell>
      <Card>
        {/* step indicator */}
        <div className="mb-5 flex items-center gap-2">
          <div
            className={`h-1.5 flex-1 rounded-full ${
              step === "otp" ? "bg-brand-500" : "bg-brand-500"
            }`}
          />
          <div
            className={`h-1.5 flex-1 rounded-full ${
              step === "set-pin" ? "bg-brand-500" : "bg-slate-200 dark:bg-slate-700"
            }`}
          />
        </div>

        {step === "otp" && (
          <>
            {name ? (
              <UserAvatar name={name} avatarUrl={avatarUrl} className="mb-3 h-14 w-14 text-xl" glow />
            ) : (
              <div className="mb-3 flex h-11 w-11 items-center justify-center rounded-xl bg-gradient-to-br from-brand-400 to-brand-600 shadow-md shadow-brand-900/20">
                <MailCheck className="h-5 w-5 text-white" />
              </div>
            )}
            <h1 className="mb-1 text-xl font-bold tracking-tight text-slate-900 dark:text-slate-50">
              {name ? `Hi ${name}, set up your robot PIN` : "Set up your robot PIN"}
            </h1>
            <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
              We sent a 6-digit code to {maskEmail(email)}.{" "}
              <button type="button" onClick={requestOtp} className="font-semibold text-brand-600 dark:text-brand-400">
                Resend code
              </button>
            </p>
            <form onSubmit={otpForm.handleSubmit(onSubmitOtp)} className="flex flex-col gap-4">
              <Input
                label="6-digit code"
                inputMode="numeric"
                maxLength={6}
                autoFocus
                error={otpForm.formState.errors.otp?.message}
                {...otpForm.register("otp")}
              />
              {error && (
                <p className="rounded-lg bg-red-50 px-3 py-2 text-sm text-red-600 dark:bg-red-900/20 dark:text-red-400">
                  {error}
                </p>
              )}
              <Button type="submit" isLoading={isSubmitting} className="mt-2">
                Verify code
              </Button>
              <Button type="button" variant="ghost" onClick={onBack}>
                <ArrowLeft className="h-4 w-4" /> Back
              </Button>
            </form>
          </>
        )}

        {step === "set-pin" && (
          <>
            <div className="mb-3 flex h-11 w-11 items-center justify-center rounded-xl bg-gradient-to-br from-brand-400 to-brand-600 shadow-md shadow-brand-900/20">
              <ShieldCheck className="h-5 w-5 text-white" />
            </div>
            <h1 className="mb-1 text-xl font-bold tracking-tight text-slate-900 dark:text-slate-50">
              Choose a 4-digit PIN
            </h1>
            <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
              You'll use this PIN to log in on this robot from now on.
            </p>
            <form onSubmit={pinForm.handleSubmit(onSubmitPin)} className="flex flex-col gap-4">
              <Input
                label="4-digit PIN"
                type="password"
                inputMode="numeric"
                maxLength={4}
                autoFocus
                error={pinForm.formState.errors.pin?.message}
                {...pinForm.register("pin")}
              />
              <Input
                label="Confirm PIN"
                type="password"
                inputMode="numeric"
                maxLength={4}
                error={pinForm.formState.errors.confirmPin?.message}
                {...pinForm.register("confirmPin")}
              />
              {error && (
                <p className="rounded-lg bg-red-50 px-3 py-2 text-sm text-red-600 dark:bg-red-900/20 dark:text-red-400">
                  {error}
                </p>
              )}
              <Button type="submit" isLoading={isSubmitting} className="mt-2">
                Set PIN
              </Button>
              <Button type="button" variant="ghost" onClick={() => setStep("otp")}>
                <ArrowLeft className="h-4 w-4" /> Back
              </Button>
            </form>
          </>
        )}
      </Card>
    </AppShell>
  );
}
