import { ReactNode, useMemo } from "react";
import { motion } from "framer-motion";

interface AppShellProps {
  children: ReactNode;
  /** Max width of the centered content card (Tailwind max-w-* class). */
  maxWidthClassName?: string;
}

const BLOBS = [
  { size: 420, top: "-18%", left: "-12%", color: "rgba(53,205,177,0.35)", duration: 16, delay: 0 },
  { size: 360, top: "55%", left: "78%", color: "rgba(18,177,152,0.3)", duration: 20, delay: -4 },
  { size: 300, top: "70%", left: "-8%", color: "rgba(10,141,124,0.3)", duration: 18, delay: -9 },
  { size: 260, top: "-10%", left: "65%", color: "rgba(112,228,205,0.3)", duration: 22, delay: -13 },
  { size: 200, top: "25%", left: "35%", color: "rgba(18,177,152,0.18)", duration: 14, delay: -6 },
];

const PARTICLE_COUNT = 18;

/**
 * Full-canvas kiosk background shared by every screen: a layered scene of
 * drifting brand-colored blobs, a rotating conic glow, a faint dot grid, and
 * slow-rising particles — behind a centered glass card. Each page just
 * supplies its content instead of re-declaring the backdrop.
 */
export function AppShell({ children, maxWidthClassName = "max-w-sm" }: AppShellProps) {
  const particles = useMemo(
    () =>
      Array.from({ length: PARTICLE_COUNT }, (_, i) => ({
        id: i,
        left: Math.random() * 100,
        size: 2 + Math.random() * 3,
        duration: 10 + Math.random() * 14,
        delay: Math.random() * -20,
        drift: (Math.random() - 0.5) * 60,
      })),
    [],
  );

  return (
    <div className="app-canvas relative flex h-screen w-full items-center justify-center overflow-hidden px-4">
      {/* slow-rotating conic halo behind the card */}
      <motion.div
        aria-hidden
        className="absolute left-1/2 top-1/2 h-[900px] w-[900px] -translate-x-1/2 -translate-y-1/2 opacity-40 dark:opacity-25"
        style={{
          background:
            "conic-gradient(from 0deg, rgba(18,177,152,0.25), transparent 25%, transparent 50%, rgba(53,205,177,0.25) 75%, rgba(18,177,152,0.25))",
          filter: "blur(60px)",
        }}
        animate={{ rotate: 360 }}
        transition={{ duration: 60, repeat: Infinity, ease: "linear" }}
      />

      {/* faint dot grid, faded toward the edges */}
      <div
        aria-hidden
        className="absolute inset-0 opacity-[0.07] dark:opacity-[0.06]"
        style={{
          backgroundImage: "radial-gradient(currentColor 1px, transparent 1px)",
          backgroundSize: "28px 28px",
          color: "#0a8d7c",
          maskImage: "radial-gradient(ellipse 70% 70% at 50% 50%, black 30%, transparent 75%)",
          WebkitMaskImage: "radial-gradient(ellipse 70% 70% at 50% 50%, black 30%, transparent 75%)",
        }}
      />

      {/* drifting color blobs */}
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
            animationDelay: `${blob.delay}s`,
          }}
        />
      ))}

      {/* slow-rising ambient particles */}
      {particles.map((p) => (
        <motion.span
          key={p.id}
          aria-hidden
          className="absolute bottom-0 rounded-full bg-brand-400/40 dark:bg-brand-300/30"
          style={{ left: `${p.left}%`, width: p.size, height: p.size }}
          animate={{
            y: ["0vh", "-90vh"],
            x: [0, p.drift],
            opacity: [0, 0.8, 0],
          }}
          transition={{
            duration: p.duration,
            delay: p.delay,
            repeat: Infinity,
            ease: "linear",
          }}
        />
      ))}

      {/* soft vignette to keep the card readable over the scene */}
      <div
        aria-hidden
        className="pointer-events-none absolute inset-0"
        style={{
          background:
            "radial-gradient(ellipse 60% 55% at 50% 50%, transparent 40%, rgba(243,246,248,0.6) 90%)",
        }}
      />
      <div
        aria-hidden
        className="pointer-events-none absolute inset-0 hidden dark:block"
        style={{
          background:
            "radial-gradient(ellipse 60% 55% at 50% 50%, transparent 40%, rgba(7,11,13,0.7) 90%)",
        }}
      />

      <motion.div
        initial={{ opacity: 0, y: 16, scale: 0.98 }}
        animate={{ opacity: 1, y: 0, scale: 1 }}
        transition={{ duration: 0.35, ease: "easeOut" }}
        className={`glass-card relative w-full ${maxWidthClassName}`}
      >
        {children}
      </motion.div>
    </div>
  );
}
