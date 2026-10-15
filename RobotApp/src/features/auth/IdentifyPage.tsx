import { useState } from "react";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { Button } from "../../components/ui/Button";
import { Card } from "../../components/ui/Card";
import { Input } from "../../components/ui/Input";
import { getApiErrorMessage } from "../../api/apiError";
import { hmiClient } from "../../api/hmiClient";
import { IdentifyFormValues, identifyFormSchema } from "./auth.schemas";

interface IdentifyPageProps {
  onIdentified: (email: string, pinSet: boolean, name?: string, avatarUrl?: string) => void;
  onBack: () => void;
}

/** First screen: who are you? Branches to PIN login or the PIN-setup wizard. */
export function IdentifyPage({ onIdentified, onBack }: IdentifyPageProps) {
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [noAccount, setNoAccount] = useState(false);
  const form = useForm<IdentifyFormValues>({ resolver: zodResolver(identifyFormSchema) });

  async function onSubmit(values: IdentifyFormValues) {
    setError(null);
    setNoAccount(false);
    setIsSubmitting(true);
    try {
      const result = await hmiClient.identify(values.email);
      if (result.status === "no-account") {
        setNoAccount(true);
        return;
      }
      if (result.status === "no-access") {
        setError("This account doesn't have access to this robot yet. Contact your admin to request access.");
        return;
      }
      onIdentified(values.email, result.pinSet, result.name, result.avatarUrl);
    } catch (err) {
      setError(getApiErrorMessage(err));
    } finally {
      setIsSubmitting(false);
    }
  }

  return (
    <div className="flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <Card className="w-full max-w-sm">
        <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">Welcome</h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
          Enter your AutoNav account email to continue.
        </p>
        <form onSubmit={form.handleSubmit(onSubmit)} className="flex flex-col gap-4">
          <Input
            label="Email"
            type="email"
            autoComplete="email"
            autoFocus
            error={form.formState.errors.email?.message}
            {...form.register("email")}
          />
          {noAccount && (
            <p className="text-sm text-red-600 dark:text-red-400">
              No account found for this email.{" "}
              <a
                href="https://autonav.in/"
                target="_blank"
                rel="noreferrer"
                className="font-medium underline"
              >
                Create one at autonav.in
              </a>
              .
            </p>
          )}
          {error && <p className="text-sm text-red-600 dark:text-red-400">{error}</p>}
          <Button type="submit" isLoading={isSubmitting} className="mt-2">
            Continue
          </Button>
          <Button type="button" variant="ghost" onClick={onBack}>
            Back
          </Button>
        </form>
      </Card>
    </div>
  );
}
