import { Link, useNavigate } from "react-router-dom";
import { Rocket } from "lucide-react";
import { Card } from "../../../components/ui/Card";
import { AuthShell } from "../../../components/ui/AuthShell";
import { ThemeToggle } from "../../../components/ui/ThemeToggle";
import { SignupForm } from "../components/SignupForm";
import { GoogleAuthButton } from "../components/GoogleAuthButton";

export function SignupPage() {
  const navigate = useNavigate();

  return (
    <AuthShell>
      <div className="absolute right-4 top-4 z-10">
        <ThemeToggle />
      </div>
      <Card className="border-0 bg-transparent p-7 shadow-none dark:bg-transparent">
        <span className="mb-4 flex h-11 w-11 items-center justify-center rounded-xl bg-gradient-to-br from-brand-400 to-brand-600 shadow-md shadow-brand-900/20">
          <Rocket className="h-5 w-5 text-white" />
        </span>
        <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">
          Create your account
        </h1>
        <p className="mb-6 text-sm text-slate-500 dark:text-slate-400">
          Get started operating your AutoNav robot.
        </p>

        <SignupForm onSuccess={() => navigate("/")} />

        <div className="my-4 flex items-center gap-2 text-xs text-slate-400 dark:text-slate-500">
          <div className="h-px flex-1 bg-slate-200 dark:bg-slate-700" />
          OR
          <div className="h-px flex-1 bg-slate-200 dark:bg-slate-700" />
        </div>

        <GoogleAuthButton />

        <p className="mt-6 text-center text-sm text-slate-500 dark:text-slate-400">
          Already have an account?{" "}
          <Link to="/login" className="font-medium text-brand-600 hover:text-brand-700 dark:text-brand-400">
            Sign in
          </Link>
        </p>
      </Card>
    </AuthShell>
  );
}
