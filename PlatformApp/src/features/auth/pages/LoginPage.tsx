import { Link, useNavigate } from "react-router-dom";
import { Card } from "../../../components/ui/Card";
import { ThemeToggle } from "../../../components/ui/ThemeToggle";
import { LoginForm } from "../components/LoginForm";
import { GoogleAuthButton } from "../components/GoogleAuthButton";

export function LoginPage() {
  const navigate = useNavigate();

  return (
    <div className="relative flex min-h-screen items-center justify-center bg-slate-50 px-4 dark:bg-slate-900">
      <div
        className="pointer-events-none absolute inset-x-0 top-0 h-80 bg-gradient-to-b from-brand-100/70 to-transparent dark:from-brand-900/20"
        aria-hidden
      />
      <div className="absolute right-4 top-4">
        <ThemeToggle />
      </div>
      <Card className="relative w-full max-w-sm">
        <span className="mb-4 flex h-10 w-10 items-center justify-center rounded-lg bg-brand-600 text-base font-bold text-white">
          A
        </span>
        <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
          Sign in to AutoNav
        </h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
          Operate and monitor your robot fleet.
        </p>

        <LoginForm onSuccess={() => navigate("/")} />

        <p className="mt-3 text-right text-sm">
          <Link to="/forgot-password" className="font-medium text-brand-600 hover:text-brand-700 dark:text-brand-400">
            Forgot password?
          </Link>
        </p>

        <div className="my-4 flex items-center gap-2 text-xs text-slate-400 dark:text-slate-500">
          <div className="h-px flex-1 bg-slate-200 dark:bg-slate-700" />
          OR
          <div className="h-px flex-1 bg-slate-200 dark:bg-slate-700" />
        </div>

        <GoogleAuthButton />

        <p className="mt-6 text-center text-sm text-slate-500 dark:text-slate-400">
          Don&apos;t have an account?{" "}
          <Link to="/signup" className="font-medium text-brand-600 hover:text-brand-700 dark:text-brand-400">
            Sign up
          </Link>
        </p>
      </Card>
    </div>
  );
}
