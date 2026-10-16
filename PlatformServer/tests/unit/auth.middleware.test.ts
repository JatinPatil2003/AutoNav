import { Request } from "express";
import { extractAccessToken } from "../../src/common/middleware/auth.middleware";

function fakeRequest(overrides: Partial<Request>): Request {
  return { cookies: {}, headers: {}, ...overrides } as unknown as Request;
}

describe("extractAccessToken", () => {
  it("prefers the accessToken cookie when present", () => {
    const req = fakeRequest({
      cookies: { accessToken: "cookie-token" },
      headers: { authorization: "Bearer header-token" },
    });
    expect(extractAccessToken(req)).toBe("cookie-token");
  });

  it("falls back to the Authorization Bearer header", () => {
    const req = fakeRequest({ headers: { authorization: "Bearer header-token" } });
    expect(extractAccessToken(req)).toBe("header-token");
  });

  it("falls back to Sec-WebSocket-Protocol for WS upgrades with no cookie or Authorization header", () => {
    const req = fakeRequest({ headers: { "sec-websocket-protocol": "bearer, ws-token" } });
    expect(extractAccessToken(req)).toBe("ws-token");
  });

  it("ignores a Sec-WebSocket-Protocol header that isn't a bearer pair", () => {
    const req = fakeRequest({ headers: { "sec-websocket-protocol": "graphql-ws" } });
    expect(extractAccessToken(req)).toBeUndefined();
  });

  it("returns undefined when no token source is present", () => {
    const req = fakeRequest({});
    expect(extractAccessToken(req)).toBeUndefined();
  });
});
