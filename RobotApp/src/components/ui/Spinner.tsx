export function FullPageSpinner() {
  return (
    <div className="app-canvas flex h-screen w-full items-center justify-center">
      <div className="flex flex-col items-center gap-4">
        <span className="h-12 w-12 animate-spin rounded-full border-4 border-brand-500 border-t-transparent" />
        <span className="text-sm font-medium text-slate-500 dark:text-slate-400">Starting up…</span>
      </div>
    </div>
  );
}
