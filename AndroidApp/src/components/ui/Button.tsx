import { ButtonHTMLAttributes, forwardRef } from "react";

type Variant = "primary" | "secondary" | "danger" | "ghost";

interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: Variant;
  isLoading?: boolean;
}

const variantClasses: Record<Variant, string> = {
  primary:
    "bg-gradient-to-b from-brand-400 to-brand-600 text-white shadow-md shadow-brand-900/20 hover:from-brand-400 hover:to-brand-700 hover:shadow-lg hover:shadow-brand-900/25 focus-visible:outline-brand-600",
  secondary:
    "bg-slate-100/80 text-slate-900 hover:bg-slate-200 focus-visible:outline-slate-400 dark:bg-slate-700/70 dark:text-slate-100 dark:hover:bg-slate-600",
  danger:
    "bg-gradient-to-b from-red-500 to-red-600 text-white shadow-md shadow-red-900/20 hover:from-red-500 hover:to-red-700 focus-visible:outline-red-600",
  ghost:
    "bg-transparent text-slate-700 hover:bg-slate-100 focus-visible:outline-slate-400 dark:text-slate-300 dark:hover:bg-slate-800",
};

export const Button = forwardRef<HTMLButtonElement, ButtonProps>(
  ({ variant = "primary", isLoading, disabled, className = "", children, ...rest }, ref) => (
    <button
      ref={ref}
      disabled={disabled || isLoading}
      className={`inline-flex items-center justify-center gap-2 rounded-xl px-4 py-2 text-sm font-medium transition-all disabled:cursor-not-allowed disabled:opacity-60 focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 active:scale-[0.97] ${variantClasses[variant]} ${className}`}
      {...rest}
    >
      {isLoading && (
        <span className="h-4 w-4 animate-spin rounded-full border-2 border-current border-t-transparent" />
      )}
      {children}
    </button>
  ),
);

Button.displayName = "Button";
