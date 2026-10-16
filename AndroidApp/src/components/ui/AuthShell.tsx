import { ReactNode } from "react";

interface AuthShellProps {
  children: ReactNode;
  maxWidthClassName?: string;
}

const BLOBS = [
  { size: 360, top: "-15%", left: "-10%", color: "rgba(53,205,177,0.3)", duration: 16 },
  { size: 300, top: "60%", left: "70%", color: "rgba(18,177,152,0.26)", duration: 20 },
  { size: 260, top: "70%", left: "-10%", color: "rgba(10,141,124,0.26)", duration: 24 },
];

/**
 * Adapted from PlatformApp's AuthShell: the same drifting brand-blob
 * backdrop behind a centered glass card, reimplemented with pure CSS
 * keyframes (see `.animate-blob` in tailwind.css) instead of framer-motion
 * — one fewer dependency for a WebView that may run on modest Android
 * hardware, for a flourish that doesn't need a full animation library.
 */
export function AuthShell({ children, maxWidthClassName = "max-w-sm" }: AuthShellProps) {
  return (
    <div className="app-canvas relative flex min-h-screen w-full items-center justify-center overflow-hidden px-4">
      {BLOBS.map((blob, i) => (
        <div
          key={i}
          aria-hidden
          className="animate-blob absolute rounded-full blur-3xl"
          style={{
            width: blob.size,
            height: blob.size,
            top: blob.top,
            left: blob.left,
            backgroundColor: blob.color,
            animationDuration: `${blob.duration}s`,
          }}
        />
      ))}

      <div className={`glass-card relative w-full ${maxWidthClassName}`}>{children}</div>
    </div>
  );
}
