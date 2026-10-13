import { createBrowserRouter } from "react-router-dom";
import { LoginPage } from "../features/auth/pages/LoginPage";
import { SignupPage } from "../features/auth/pages/SignupPage";
import { OAuthCallbackPage } from "../features/auth/pages/OAuthCallbackPage";
import { ForgotPasswordPage } from "../features/auth/pages/ForgotPasswordPage";
import { RequestAccessPage } from "../features/robot-access/pages/RequestAccessPage";
import { AdminRequestsPage } from "../features/admin/pages/AdminRequestsPage";
import { AdminUsersPage } from "../features/admin/pages/AdminUsersPage";
import { DashboardLayout } from "../features/dashboard/layout/DashboardLayout";
import { DashboardHome } from "../features/dashboard/pages/DashboardHome";
import { MappingPlaceholderPage } from "../features/dashboard/pages/MappingPlaceholderPage";
import { NavigationPlaceholderPage } from "../features/dashboard/pages/NavigationPlaceholderPage";
import { AnalyticsPlaceholderPage } from "../features/dashboard/pages/AnalyticsPlaceholderPage";
import { DocumentationPlaceholderPage } from "../features/dashboard/pages/DocumentationPlaceholderPage";
import { PublicRoute } from "./PublicRoute";
import { ProtectedRoute } from "./ProtectedRoute";
import { AdminRoute } from "./AdminRoute";
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
    path: "/auth/callback",
    element: <OAuthCallbackPage />,
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
    path: "/admin/requests",
    element: (
      <AdminRoute>
        <AdminRequestsPage />
      </AdminRoute>
    ),
  },
  {
    path: "/admin/users",
    element: (
      <AdminRoute>
        <AdminUsersPage />
      </AdminRoute>
    ),
  },
  {
    path: "/",
    element: (
      <RobotRequiredRoute>
        <DashboardLayout />
      </RobotRequiredRoute>
    ),
    children: [
      { index: true, element: <DashboardHome /> },
      { path: "mapping", element: <MappingPlaceholderPage /> },
      { path: "navigation", element: <NavigationPlaceholderPage /> },
      { path: "analytics", element: <AnalyticsPlaceholderPage /> },
      { path: "docs", element: <DocumentationPlaceholderPage /> },
    ],
  },
]);
