import { Construction } from "lucide-react";

export function PlaceholderPage({ title, subtitle }: { title: string; subtitle?: string }) {
  return (
    <div className="p-3">
      <div className="glass-panel flex flex-col items-center justify-center gap-3 rounded-2xl px-6 py-16 text-center">
        <span className="flex h-12 w-12 items-center justify-center rounded-xl bg-gradient-to-br from-brand-400 to-brand-600 shadow-md shadow-brand-900/20">
          <Construction className="h-6 w-6 text-white" />
        </span>
        <h1 className="text-xl font-semibold text-slate-900 dark:text-slate-100">{title}</h1>
        {subtitle && <p className="text-sm text-slate-500 dark:text-slate-400">{subtitle}</p>}
        <p className="text-sm text-slate-400 dark:text-slate-500">This feature is coming in a future milestone.</p>
      </div>
    </div>
  );
}
