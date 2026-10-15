export function PlaceholderPage({ title, subtitle }: { title: string; subtitle?: string }) {
  return (
    <div className="p-6">
      <h1 className="mb-1 text-xl font-semibold text-slate-900 dark:text-slate-100">{title}</h1>
      {subtitle && <p className="mb-2 text-sm text-slate-500 dark:text-slate-400">{subtitle}</p>}
      <p className="text-sm text-slate-500 dark:text-slate-400">
        This feature is coming in a future milestone.
      </p>
    </div>
  );
}
