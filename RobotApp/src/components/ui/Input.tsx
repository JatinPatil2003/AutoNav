import { InputHTMLAttributes, forwardRef } from "react";

interface InputProps extends InputHTMLAttributes<HTMLInputElement> {
  label: string;
  error?: string;
}

export const Input = forwardRef<HTMLInputElement, InputProps>(
  ({ label, error, id, className = "", ...rest }, ref) => {
    const inputId = id ?? label.toLowerCase().replace(/\s+/g, "-");

    return (
      <div className="flex flex-col gap-1.5">
        <label htmlFor={inputId} className="text-sm font-medium text-slate-600 dark:text-slate-300">
          {label}
        </label>
        <input
          ref={ref}
          id={inputId}
          className={`rounded-xl border bg-white/80 px-3.5 py-2.5 text-sm text-slate-900 shadow-sm ring-1 ring-transparent transition-all focus:outline-none focus:ring-2 focus:ring-brand-500 dark:bg-slate-900/60 dark:text-slate-100 ${
            error ? "border-red-400" : "border-slate-200 dark:border-slate-700"
          } ${className}`}
          {...rest}
        />
        {error && <p className="text-xs font-medium text-red-500 dark:text-red-400">{error}</p>}
      </div>
    );
  },
);

Input.displayName = "Input";
