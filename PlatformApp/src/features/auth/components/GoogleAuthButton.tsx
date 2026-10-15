import { googleLoginUrl } from "../authApi";

export function GoogleAuthButton() {
  return (
    <a
      href={googleLoginUrl()}
      className="flex w-full items-center justify-center gap-2 rounded-md border border-slate-300 bg-white px-4 py-2 text-sm font-medium text-slate-700 shadow-sm transition-colors hover:bg-slate-50 dark:border-slate-600 dark:bg-slate-800 dark:text-slate-200 dark:hover:bg-slate-700"
    >
      <svg aria-hidden="true" viewBox="0 0 24 24" className="h-4 w-4">
        <path
          fill="#4285F4"
          d="M23.52 12.27c0-.79-.07-1.54-.2-2.27H12v4.3h6.47c-.28 1.5-1.13 2.77-2.4 3.62v3.01h3.49c2.04-1.88 3.23-4.65 3.23-8.66z"
        />
        <path
          fill="#34A853"
          d="M12 24c3.24 0 5.95-1.07 7.93-2.9l-3.49-3.01c-1.03.7-2.4 1.1-4.44 1.1-3.42 0-6.32-2.3-7.35-5.4H1.07v3.1C3.03 21.3 7.2 24 12 24z"
        />
        <path
          fill="#FBBC05"
          d="M4.65 14.79c-.26-.78-.4-1.62-.4-2.49s.14-1.71.4-2.49V6.72H1.07A11.98 11.98 0 0 0 0 12.3c0 1.94.46 3.78 1.07 5.4z"
        />
        <path
          fill="#EA4335"
          d="M12 4.75c1.76 0 3.34.6 4.58 1.78l3.1-3.1C17.94 1.7 15.23.6 12 .6 7.2.6 3.03 3.3 1.07 6.72l3.58 2.78c1.03-3.1 3.93-5.4 7.35-5.4z"
        />
      </svg>
      Continue with Google
    </a>
  );
}
