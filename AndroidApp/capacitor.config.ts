import { readFileSync } from "node:fs";
import { join } from "node:path";
import type { CapacitorConfig } from "@capacitor/cli";

/**
 * The production backend is TLS-terminated end-to-end (nginx), so mixed
 * content is correctly disallowed by default. Local dev against a plain
 * `http://<lan-ip>:4000` backend (no TLS, since that's dev-only) would
 * otherwise have every request silently blocked by the WebView's mixed
 * content policy (the page itself loads over `https://localhost`). Rather
 * than hand-toggle this and risk shipping it `true` by accident, derive it
 * from the actual scheme in `.env`'s `VITE_API_BASE_URL` at build time.
 */
function isDevHttpBackend(): boolean {
  try {
    const env = readFileSync(join(__dirname, ".env"), "utf-8");
    const match = env.match(/^VITE_API_BASE_URL=(.*)$/m);
    return match?.[1].trim().startsWith("http://") ?? false;
  } catch {
    return false;
  }
}

const devHttpBackend = isDevHttpBackend();

const config: CapacitorConfig = {
  appId: "in.autonav.app",
  appName: "AutoNav",
  webDir: "dist",
  android: {
    allowMixedContent: devHttpBackend,
  },
  // Default `https://localhost` + an http:// backend triggers Chromium's
  // mixed-content *autoupgrade* (rewrites the request to https:// before
  // even consulting allowMixedContent), which then fails outright since
  // the dev backend has no TLS listener — appearing as a generic network
  // error with no response body. Loading the app itself over plain
  // `http://localhost` in that case sidesteps the secure/insecure
  // mismatch entirely, so there's nothing to autoupgrade.
  ...(devHttpBackend ? { server: { androidScheme: "http" as const } } : {}),
};

export default config;
