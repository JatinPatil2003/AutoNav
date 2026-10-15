import { useState } from "react";

interface UserAvatarProps {
  name: string;
  avatarUrl?: string;
  className?: string;
}

/** Shows the user's avatarUrl when set; falls back to an initial on a solid circle, including on image load failure. */
export function UserAvatar({ name, avatarUrl, className = "h-9 w-9 text-sm" }: UserAvatarProps) {
  const [imageFailed, setImageFailed] = useState(false);

  if (avatarUrl && !imageFailed) {
    return (
      <img
        src={avatarUrl}
        alt=""
        onError={() => setImageFailed(true)}
        className={`flex-shrink-0 rounded-full object-cover ${className}`}
      />
    );
  }

  return (
    <span
      className={`flex flex-shrink-0 items-center justify-center rounded-full bg-brand-600 font-semibold text-white ${className}`}
    >
      {name.charAt(0).toUpperCase()}
    </span>
  );
}
