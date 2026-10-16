interface AvatarProps {
  name?: string;
  email?: string;
  avatarUrl?: string;
  size?: number;
  className?: string;
}

function getInitials(value?: string): string {
  if (!value) return "?";
  const parts = value.trim().split(/\s+/);
  if (parts.length === 1) return parts[0].slice(0, 2).toUpperCase();
  return (parts[0][0] + parts[parts.length - 1][0]).toUpperCase();
}

/** A photo avatar when `avatarUrl` is set (e.g. Google sign-in), otherwise initials on a brand gradient. */
export function Avatar({ name, email, avatarUrl, size = 56, className = "" }: AvatarProps) {
  const dimension = { width: size, height: size };

  if (avatarUrl) {
    return (
      <img
        src={avatarUrl}
        alt={name ?? email ?? "Profile"}
        style={dimension}
        className={`flex-shrink-0 rounded-full object-cover ring-2 ring-white/80 dark:ring-slate-800 ${className}`}
      />
    );
  }

  return (
    <span
      style={{ ...dimension, fontSize: size * 0.36 }}
      className={`flex flex-shrink-0 items-center justify-center rounded-full bg-gradient-to-br from-brand-400 to-brand-600 font-semibold text-white shadow-md shadow-brand-900/20 ring-2 ring-white/80 dark:ring-slate-800 ${className}`}
    >
      {getInitials(name ?? email)}
    </span>
  );
}
