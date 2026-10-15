/**
 * Minimal cookie-header parser for the raw WebSocket upgrade path, which
 * runs before any Express middleware (including cookie-parser) sees the
 * request — see core/wsRouter.ts's authenticate callback.
 */
export function parseCookies(cookieHeader: string | undefined): Record<string, string> {
  if (!cookieHeader) return {};

  return Object.fromEntries(
    cookieHeader
      .split(";")
      .map((pair) => pair.trim())
      .filter(Boolean)
      .map((pair) => {
        const index = pair.indexOf("=");
        if (index === -1) return [pair, ""];
        const name = pair.slice(0, index).trim();
        const value = decodeURIComponent(pair.slice(index + 1).trim());
        return [name, value];
      }),
  );
}
