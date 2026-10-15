import { useEffect, useState } from "react";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { Input } from "../../components/ui/Input";
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
    <div className="flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <Card className="w-full max-w-sm">
        {step === "otp" && (
          <>
            {name && <UserAvatar name={name} avatarUrl={avatarUrl} className="mb-3 h-12 w-12 text-lg" />}
            <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
              {name ? `Hi ${name}, set up your robot PIN` : "Set up your robot PIN"}
            </h1>
            <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
              We sent a 6-digit code to {maskEmail(email)}.{" "}
              <button type="button" onClick={requestOtp} className="font-medium text-brand-600 dark:text-brand-400">
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
              {error && <p className="text-sm text-red-600 dark:text-red-400">{error}</p>}
              <Button type="submit" isLoading={isSubmitting} className="mt-2">
                Verify code
              </Button>
              <Button type="button" variant="ghost" onClick={onBack}>
                Back
              </Button>
            </form>
          </>
        )}

        {step === "set-pin" && (
          <>
            <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
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
              {error && <p className="text-sm text-red-600 dark:text-red-400">{error}</p>}
              <Button type="submit" isLoading={isSubmitting} className="mt-2">
                Set PIN
              </Button>
              <Button type="button" variant="ghost" onClick={() => setStep("otp")}>
                Back
              </Button>
            </form>
          </>
        )}
      </Card>
    </div>
  );
}
