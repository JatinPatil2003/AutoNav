import type { Config } from "tailwindcss";

export default {
  // Dark mode is a nice-to-have here (no persisted-toggle UI in this app,
  // unlike PlatformApp) — "media" lets the existing dark: utility classes
  // on copied components follow the OS theme automatically, with no
  // ThemeProvider/localStorage plumbing needed.
  darkMode: "media",
  content: ["./index.html", "./src/**/*.{ts,tsx}"],
  theme: {
    extend: {
      colors: {
        // Same brand scale as PlatformApp, kept in sync by hand.
        brand: {
          50: "#eefdf9",
          100: "#d4faf0",
          200: "#aaf2e2",
          300: "#70e4cd",
          400: "#35cdb1",
          500: "#12b198",
          600: "#0a8d7c",
          700: "#0b7066",
          800: "#0d5a53",
          900: "#0e4a45",
          950: "#052b29",
        },
      },
      fontFamily: {
        sans: ["Inter", "ui-sans-serif", "system-ui", "sans-serif"],
      },
      spacing: {
        "safe-top": "env(safe-area-inset-top)",
        "safe-bottom": "env(safe-area-inset-bottom)",
        "safe-left": "env(safe-area-inset-left)",
        "safe-right": "env(safe-area-inset-right)",
      },
    },
  },
  plugins: [],
} satisfies Config;
