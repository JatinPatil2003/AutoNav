import type { Config } from "tailwindcss";

export default {
  darkMode: "class",
  content: ["./index.html", "./src/**/*.{ts,tsx}"],
  theme: {
    extend: {
      colors: {
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
    },
  },
  plugins: [],
} satisfies Config;
