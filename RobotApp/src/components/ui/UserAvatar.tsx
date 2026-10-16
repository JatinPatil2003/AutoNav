import { useState } from "react";

interface UserAvatarProps {
  name: string;
  avatarUrl?: string;
  className?: string;
  /** Adds a soft brand-colored glow ring, for hero/header placements. */
  glow?: boolean;
}

/** Shows the user's avatarUrl when set; falls back to an initial on a gradient circle, including on image load failure. */
export function UserAvatar({
  name,
  avatarUrl,
  className = "h-9 w-9 text-sm",
  glow = false,
}: UserAvatarProps) {
  const [imageFailed, setImageFailed] = useState(false);
  const ring = glow ? "glow-ring ring-2 ring-white/80 dark:ring-white/10" : "";

  if (avatarUrl && !imageFailed) {
    return (
      <img
        src={avatarUrl}
        alt=""
        onError={() => setImageFailed(true)}
        className={`flex-shrink-0 rounded-full object-cover ${ring} ${className}`}
      />
    );
  }

  return (
    <span
      className={`flex flex-shrink-0 items-center justify-center rounded-full bg-gradient-to-br from-brand-400 to-brand-600 font-semibold text-white ${ring} ${className}`}
    >
      {name.charAt(0).toUpperCase()}
    </span>
  );
}
