import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5190,
    // Allows a physical device on the same LAN to reach the dev server
    // when VITE_API_BASE_URL also points at the dev machine's LAN IP.
    host: true,
  },
});
