export function FullPageSpinner() {
  return (
    <div className="flex h-screen w-full items-center justify-center bg-white dark:bg-slate-900">
      <span className="h-10 w-10 animate-spin rounded-full border-4 border-brand-600 border-t-transparent" />
    </div>
  );
}
