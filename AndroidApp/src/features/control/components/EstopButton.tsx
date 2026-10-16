import { Hand, OctagonAlert } from "lucide-react";

interface EstopButtonProps {
  engaged: boolean;
  disabled?: boolean;
  onEngage: () => void;
  onRelease: () => void;
}

/**
 * A large circular button (not the full-width bar this used to be) so it
 * can sit opposite the joystick in ControlPage's bottom-corner layout —
 * both within thumb reach while holding the phone one-handed.
 */
export function EstopButton({ engaged, disabled, onEngage, onRelease }: EstopButtonProps) {
  const Icon = engaged ? Hand : OctagonAlert;

  return (
    <div className="flex flex-col items-center gap-2">
      <button
        type="button"
        disabled={disabled && !engaged}
        onClick={engaged ? onRelease : onEngage}
        className={`flex h-20 w-20 items-center justify-center rounded-full shadow-xl transition-all active:scale-95 disabled:cursor-not-allowed disabled:opacity-50 ${
          engaged
            ? "bg-gradient-to-b from-slate-100 to-slate-300 text-slate-700 shadow-slate-900/15 ring-4 ring-amber-400/70 dark:from-slate-700 dark:to-slate-800 dark:text-slate-100"
            : "bg-gradient-to-b from-red-500 to-red-600 text-white shadow-red-900/30"
        }`}
      >
        <Icon className="h-8 w-8" />
      </button>
      <span className="text-xs font-medium text-slate-500 dark:text-slate-400">
        {engaged ? "Tap to resume" : "Emergency stop"}
      </span>
    </div>
  );
}
