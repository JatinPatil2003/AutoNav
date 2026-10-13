import { describe, expect, it, vi } from "vitest";
import { screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { renderWithProviders } from "../../../test/renderWithProviders";
import { LoginForm } from "./LoginForm";

describe("LoginForm", () => {
  it("shows validation errors instead of submitting when fields are invalid", async () => {
    const onSuccess = vi.fn();
    renderWithProviders(<LoginForm onSuccess={onSuccess} />);

    await userEvent.click(screen.getByRole("button", { name: /sign in/i }));

    expect(await screen.findByText(/enter a valid email/i)).toBeInTheDocument();
    expect(onSuccess).not.toHaveBeenCalled();
  });

  it("lets the user type into email and password fields", async () => {
    renderWithProviders(<LoginForm onSuccess={vi.fn()} />);

    const email = screen.getByLabelText(/email/i);
    const password = screen.getByLabelText("Password", { exact: true });

    await userEvent.type(email, "user@example.com");
    await userEvent.type(password, "secret123");

    await waitFor(() => {
      expect(email).toHaveValue("user@example.com");
      expect(password).toHaveValue("secret123");
    });
  });
});
