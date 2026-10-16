import { Navigate, createBrowserRouter } from "react-router-dom";
import { ForgotPasswordPage } from "../features/auth/pages/ForgotPasswordPage";
import { LoginPage } from "../features/auth/pages/LoginPage";
import { SignupPage } from "../features/auth/pages/SignupPage";
import { AnalyticsPlaceholderPage } from "../features/analytics/pages/AnalyticsPlaceholderPage";
import { ControlPage } from "../features/control/pages/ControlPage";
import { HomeShell } from "../features/home/HomeShell";
import { RequestAccessPage } from "../features/robot-access/pages/RequestAccessPage";
import { RobotSelectPage } from "../features/robot-select/pages/RobotSelectPage";
import { SettingsPage } from "../features/settings/pages/SettingsPage";
import { ProtectedRoute } from "./ProtectedRoute";
import { PublicRoute } from "./PublicRoute";
import { RobotRequiredRoute } from "./RobotRequiredRoute";

export const router = createBrowserRouter([
  {
    path: "/login",
    element: (
      <PublicRoute>
        <LoginPage />
      </PublicRoute>
    ),
  },
  {
    path: "/signup",
    element: (
      <PublicRoute>
        <SignupPage />
      </PublicRoute>
    ),
  },
  {
    path: "/forgot-password",
    element: (
      <PublicRoute>
        <ForgotPasswordPage />
      </PublicRoute>
    ),
  },
  {
    path: "/request-access",
    element: (
      <ProtectedRoute>
        <RequestAccessPage />
      </ProtectedRoute>
    ),
  },
  {
    path: "/robot-select",
    element: (
      <ProtectedRoute>
        <RobotSelectPage />
      </ProtectedRoute>
    ),
  },
  {
    path: "/",
    element: (
      <RobotRequiredRoute>
        <HomeShell />
      </RobotRequiredRoute>
    ),
    children: [
      { index: true, element: <Navigate to="control" replace /> },
      { path: "control", element: <ControlPage /> },
      { path: "analytics", element: <AnalyticsPlaceholderPage /> },
      { path: "settings", element: <SettingsPage /> },
    ],
  },
]);

/** Exported so native/backButton.ts can type its parameter without reaching into react-router's internals. */
export type AppRouter = typeof router;
