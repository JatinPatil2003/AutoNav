import { PropsWithChildren, useCallback, useEffect, useState } from "react";
import { Theme, ThemeContext } from "./theme.context";

const STORAGE_KEY = "autonav-theme";

function getInitialTheme(): Theme {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (stored === "light" || stored === "dark") return stored;
  } catch {
    // localStorage unavailable (e.g. private browsing) — fall through to system preference
  }
  return window.matchMedia?.("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}

function applyTheme(theme: Theme) {
  document.documentElement.classList.toggle("dark", theme === "dark");
}

/**
 * Tracks light/dark in state + localStorage and toggles the `dark` class on
 * <html>, which Tailwind's `darkMode: "class"` strategy reads. The initial
 * class is also set synchronously in index.html (before React loads) to
 * avoid a light-mode flash on first paint.
 */
export function ThemeProvider({ children }: PropsWithChildren) {
  const [theme, setTheme] = useState<Theme>(getInitialTheme);

  useEffect(() => {
    applyTheme(theme);
    try {
      localStorage.setItem(STORAGE_KEY, theme);
    } catch {
      // ignore write failures (private browsing, storage disabled, etc.)
    }
  }, [theme]);

  const toggleTheme = useCallback(() => {
    setTheme((t) => (t === "dark" ? "light" : "dark"));
  }, []);

  return <ThemeContext.Provider value={{ theme, toggleTheme }}>{children}</ThemeContext.Provider>;
}
