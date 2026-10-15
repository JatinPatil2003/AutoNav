import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { Button } from "../../../components/ui/Button";
import { PasswordInput } from "../../../components/ui/PasswordInput";
import { getApiErrorMessage } from "../../../app/apiError";
import { useSetPasswordMutation } from "../authApi";
import { SetPasswordFormValues, setPasswordFormSchema } from "../auth.schemas";

/**
 * Lets a Google-only account add a local password, so it can also log in
 * via the email/password form. Hidden entirely once a password exists.
 */
export function SetPasswordForm() {
  const [setPassword, { isLoading, error, isSuccess }] = useSetPasswordMutation();
  const {
    register,
    handleSubmit,
    reset,
    formState: { errors },
  } = useForm<SetPasswordFormValues>({ resolver: zodResolver(setPasswordFormSchema) });

  async function onSubmit(values: SetPasswordFormValues) {
    try {
      await setPassword({ password: values.password }).unwrap();
      reset();
    } catch {
      // error state is surfaced below via the mutation's `error` field
    }
  }

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="flex flex-col gap-4">
      <PasswordInput
        label="New password"
        autoComplete="new-password"
        error={errors.password?.message}
        {...register("password")}
      />
      <PasswordInput
        label="Confirm password"
        autoComplete="new-password"
        error={errors.confirmPassword?.message}
        {...register("confirmPassword")}
      />
      {error && <p className="text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(error)}</p>}
      {isSuccess && (
        <p className="text-sm text-green-600 dark:text-green-400">
          Password set — you can now log in with email/password too.
        </p>
      )}
      <Button type="submit" isLoading={isLoading} className="mt-1">
        Set password
      </Button>
    </form>
  );
}
