import { NextFunction, Request, Response } from "express";
import { UnauthorizedError } from "../errors";
import { verifyAccessToken } from "../utils/jwt.util";

/**
 * Web clients carry the access token in the httpOnly cookie. Native clients
 * (no cookie jar semantics worth relying on) send it as a Bearer header
 * instead. Cookie wins if both are somehow present. Shared with the /ws/control
 * upgrade authenticator so both entry points accept a session the same way.
 *
 * A third source exists for that same WS upgrade: a browser-engine WebSocket
 * (including a Capacitor WebView — no native exemption) cannot set an
 * Authorization header on the handshake, and the token must never go in the
 * query string (leaks into proxy/access logs). `Sec-WebSocket-Protocol` *can*
 * be set from JS (`new WebSocket(url, ["bearer", token])`) and is part of the
 * WS handshake spec rather than a URL, so it's used as a bearer-token carrier
 * for that one case. REST callers never send this header, so this is purely
 * additive.
 */
export function extractAccessToken(req: Request): string | undefined {
  const cookieToken = req.cookies?.accessToken as string | undefined;
  if (cookieToken) return cookieToken;

  const header = req.headers.authorization;
  if (header?.startsWith("Bearer ")) {
    return header.slice("Bearer ".length).trim();
  }

  const wsProtocolToken = extractBearerFromWsProtocol(req.headers["sec-websocket-protocol"]);
  if (wsProtocolToken) return wsProtocolToken;

  return undefined;
}

/** Parses `Sec-WebSocket-Protocol: bearer, <token>` (comma-separated subprotocol list). */
function extractBearerFromWsProtocol(header: string | string[] | undefined): string | undefined {
  if (!header) return undefined;
  const raw = Array.isArray(header) ? header.join(",") : header;
  const parts = raw.split(",").map((part) => part.trim());
  if (parts.length < 2 || parts[0].toLowerCase() !== "bearer") return undefined;
  return parts[1] || undefined;
}

export function requireAuth(req: Request, _res: Response, next: NextFunction): void {
  const token = extractAccessToken(req);

  if (!token) {
    throw new UnauthorizedError("Authentication required");
  }

  try {
    const payload = verifyAccessToken(token);
    req.user = { id: payload.sub, role: payload.role };
    next();
  } catch {
    throw new UnauthorizedError("Invalid or expired access token");
  }
}
