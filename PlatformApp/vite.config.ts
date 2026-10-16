/// <reference types="vitest/config" />
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5175,
    // host: true + allowedHosts so the EC2 nginx dev reverse-proxy (which
    // forwards Host: dashboard.autonav.in) can reach this dev server over
    // the VPN — see docs/Development Mode - Laptop Reverse Proxy + Remote MongoDB.md.
    // Only affects `vite dev`; the production container serves a static build.
    host: true,
    allowedHosts: ["dashboard.autonav.in"],
  },
  test: {
    environment: "jsdom",
    globals: true,
    setupFiles: "./src/test/setup.ts",
    css: true,
    exclude: ["node_modules", "dist", "src/legacy-robot-ui"],
  },
});
