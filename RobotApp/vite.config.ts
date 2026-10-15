/// <reference types="vitest/config" />
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { VitePWA } from "vite-plugin-pwa";

export default defineConfig({
  plugins: [
    react(),
    VitePWA({
      registerType: "autoUpdate",
      manifest: {
        name: "AutoNav Robot",
        short_name: "AutoNav",
        description: "Local operator login for this robot",
        theme_color: "#0a8d7c",
        background_color: "#0a8d7c",
        display: "fullscreen",
        start_url: "/",
      },
    }),
  ],
  server: {
    port: 5174,
    strictPort: true,
    // Local dev only — in production this app is served same-origin by
    // RobotServer itself (see RobotServer's app.ts static mount), so no
    // proxy/CORS handling is needed there at all.
    proxy: {
      "/api": "http://localhost:8000",
    },
  },
  test: {
    environment: "jsdom",
    globals: true,
    setupFiles: "./src/test/setup.ts",
    css: true,
  },
});
