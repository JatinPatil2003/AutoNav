import { useState } from "react";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { CheckCircle2, KeyRound } from "lucide-react";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { Input } from "../../components/ui/Input";
import { AppShell } from "../../components/ui/AppShell";
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
    <AppShell>
      <Card>
        <div className="mb-5 flex items-center gap-3">
          <UserAvatar name={name ?? email} avatarUrl={avatarUrl} className="h-14 w-14 text-xl" glow />
          <div className="min-w-0">
            <h1 className="text-xl font-bold tracking-tight text-slate-900 dark:text-slate-50">
              Enter your PIN
            </h1>
            <p className="truncate text-sm text-slate-500 dark:text-slate-400">{name ?? maskEmail(email)}</p>
          </div>
        </div>
        {notice && (
          <p className="mb-4 flex items-center gap-2 rounded-lg bg-emerald-50 px-3 py-2 text-sm font-medium text-emerald-700 dark:bg-emerald-900/20 dark:text-emerald-300">
            <CheckCircle2 className="h-4 w-4 flex-shrink-0" />
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
          {error && (
            <p className="rounded-lg bg-red-50 px-3 py-2 text-sm text-red-600 dark:bg-red-900/20 dark:text-red-400">
              {error}
            </p>
          )}
          <Button type="submit" isLoading={isSubmitting} className="mt-2">
            <KeyRound className="h-4 w-4" /> Log in
          </Button>
          <button
            type="button"
            onClick={onForgotPin}
            className="text-center text-sm font-semibold text-brand-600 transition-colors hover:text-brand-700 dark:text-brand-400"
          >
            Forgot PIN?
          </button>
          <Button type="button" variant="ghost" onClick={onBack}>
            Not you?
          </Button>
        </form>
      </Card>
    </AppShell>
  );
}
