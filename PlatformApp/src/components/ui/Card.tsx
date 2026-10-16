import { HTMLAttributes } from "react";

export function Card({ className = "", children, ...rest }: HTMLAttributes<HTMLDivElement>) {
  return (
    <div
      className={`rounded-xl border border-slate-200/70 bg-white/80 p-6 shadow-sm shadow-slate-200/60 backdrop-blur-sm dark:border-slate-700/70 dark:bg-slate-800/70 dark:shadow-none ${className}`}
      {...rest}
    >
      {children}
    </div>
  );
}
