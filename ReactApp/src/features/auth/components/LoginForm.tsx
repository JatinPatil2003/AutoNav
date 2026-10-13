import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { Button } from "../../../components/ui/Button";
import { Input } from "../../../components/ui/Input";
import { PasswordInput } from "../../../components/ui/PasswordInput";
import { getApiErrorMessage } from "../../../app/apiError";
import { useGetMeQuery, useLoginMutation } from "../authApi";
import { LoginFormValues, loginFormSchema } from "../auth.schemas";

interface LoginFormProps {
  onSuccess: () => void;
}

export function LoginForm({ onSuccess }: LoginFormProps) {
  const [login, { isLoading, error }] = useLoginMutation();
  const { refetch: refetchMe } = useGetMeQuery();
  const {
    register,
    handleSubmit,
    formState: { errors },
  } = useForm<LoginFormValues>({ resolver: zodResolver(loginFormSchema) });

  async function onSubmit(values: LoginFormValues) {
    try {
      await login(values).unwrap();
      // The pre-login getMe() cache entry is a cached 401, which tag
      // invalidation alone won't refetch — force it before navigating.
      await refetchMe().unwrap();
      onSuccess();
    } catch {
      // error state is surfaced below via the mutation's `error` field
    }
  }

  return (
    <form onSubmit={handleSubmit(onSubmit)} className="flex flex-col gap-4">
      <Input
        label="Email"
        type="email"
        autoComplete="email"
        error={errors.email?.message}
        {...register("email")}
      />
      <PasswordInput
        label="Password"
        autoComplete="current-password"
        error={errors.password?.message}
        {...register("password")}
      />
      {error && <p className="text-sm text-red-600 dark:text-red-400">{getApiErrorMessage(error)}</p>}
      <Button type="submit" isLoading={isLoading} className="mt-2">
        Sign in
      </Button>
    </form>
  );
}
