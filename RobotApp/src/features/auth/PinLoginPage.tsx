import { useState } from "react";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { Input } from "../../components/ui/Input";
import { UserAvatar } from "../../components/ui/UserAvatar";
import { getApiErrorMessage } from "../../api/apiError";
import { hmiClient } from "../../api/hmiClient";
import { maskEmail } from "../../utils/maskEmail";
import { PinLoginFormValues, pinLoginFormSchema } from "./auth.schemas";

interface PinLoginPageProps {
  email: string;
  name?: string;
  avatarUrl?: string;
  onBack: () => void;
  onLoggedIn: () => void;
  onForgotPin: () => void;
  notice?: string;
}

export function PinLoginPage({
  email,
  name,
  avatarUrl,
  onBack,
  onLoggedIn,
  onForgotPin,
  notice,
}: PinLoginPageProps) {
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const form = useForm<PinLoginFormValues>({ resolver: zodResolver(pinLoginFormSchema) });

  async function onSubmit(values: PinLoginFormValues) {
    setError(null);
    setIsSubmitting(true);
    try {
      await hmiClient.login(email, values.pin);
      onLoggedIn();
    } catch (err) {
      setError(getApiErrorMessage(err));
      form.resetField("pin");
    } finally {
      setIsSubmitting(false);
    }
  }

  return (
    <div className="flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <Card className="w-full max-w-sm">
        <div className="mb-4 flex items-center gap-3">
          <UserAvatar name={name ?? email} avatarUrl={avatarUrl} className="h-12 w-12 text-lg" />
          <div>
            <h1 className="text-xl font-semibold text-slate-900 dark:text-slate-100">Enter your PIN</h1>
            <p className="text-sm text-slate-500 dark:text-slate-400">{name ?? maskEmail(email)}</p>
          </div>
        </div>
        {notice && (
          <p className="mb-4 rounded-md bg-green-50 px-3 py-2 text-sm text-green-700 dark:bg-green-900/30 dark:text-green-300">
            {notice}
          </p>
        )}
        <form onSubmit={form.handleSubmit(onSubmit)} className="flex flex-col gap-4">
          <Input
            label="4-digit PIN"
            type="password"
            inputMode="numeric"
            maxLength={4}
            autoFocus
            error={form.formState.errors.pin?.message}
            {...form.register("pin")}
          />
          {error && <p className="text-sm text-red-600 dark:text-red-400">{error}</p>}
          <Button type="submit" isLoading={isSubmitting} className="mt-2">
            Log in
          </Button>
          <button
            type="button"
            onClick={onForgotPin}
            className="text-center text-sm font-medium text-brand-600 dark:text-brand-400"
          >
            Forgot PIN?
          </button>
          <Button type="button" variant="ghost" onClick={onBack}>
            Not you?
          </Button>
        </form>
      </Card>
    </div>
  );
}
