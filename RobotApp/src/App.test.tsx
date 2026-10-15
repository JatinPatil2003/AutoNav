import { render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { beforeEach, describe, expect, it, vi } from "vitest";
import { App } from "./App";

describe("App", () => {
  beforeEach(() => {
    vi.stubGlobal(
      "fetch",
      vi.fn(async () => new Response(JSON.stringify({ message: "Unauthorized" }), { status: 401 })),
    );
  });

  it("shows the user picker when there is no existing session", async () => {
    render(<App />);

    await waitFor(() => {
      expect(screen.getByRole("heading", { name: /who's this/i })).toBeInTheDocument();
    });
  });

  it("routes to the PIN-setup wizard for a known user with no PIN yet", async () => {
    const fetchMock = vi.fn(async (input: RequestInfo | URL) => {
      const url = String(input);
      if (url.endsWith("/api/hmi-auth/session")) {
        return new Response(JSON.stringify({ message: "Unauthorized" }), { status: 401 });
      }
      if (url.endsWith("/api/hmi-auth/users")) {
        return new Response(JSON.stringify({ users: [] }), { status: 200 });
      }
      if (url.endsWith("/api/hmi-auth/identify")) {
        return new Response(JSON.stringify({ status: "has-access", pinSet: false }), { status: 200 });
      }
      if (url.endsWith("/api/hmi-auth/pin-setup/request-otp")) {
        return new Response(JSON.stringify({ message: "sent" }), { status: 200 });
      }
      throw new Error(`Unexpected fetch to ${url}`);
    });
    vi.stubGlobal("fetch", fetchMock);

    render(<App />);
    await waitFor(() => screen.getByRole("button", { name: /\+ new user/i }));
    await userEvent.click(screen.getByRole("button", { name: /\+ new user/i }));

    await waitFor(() => screen.getByLabelText(/email/i));

    await userEvent.type(screen.getByLabelText(/email/i), "newuser@example.com");
    await userEvent.click(screen.getByRole("button", { name: /continue/i }));

    await waitFor(() => {
      expect(screen.getByRole("heading", { name: /set up your robot pin/i })).toBeInTheDocument();
    });
  });
});
