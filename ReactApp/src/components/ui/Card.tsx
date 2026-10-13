import { HTMLAttributes } from "react";

export function Card({ className = "", children, ...rest }: HTMLAttributes<HTMLDivElement>) {
  return (
    <div
      className={`rounded-xl border border-slate-200 bg-white p-6 shadow-sm shadow-slate-200/60 dark:border-slate-700 dark:bg-slate-800 dark:shadow-none ${className}`}
      {...rest}
    >
      {children}
    </div>
  );
}
