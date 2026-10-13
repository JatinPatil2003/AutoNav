# Changelog

All notable changes to `ReactApp` are documented here. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Changed
- Migrated the build tooling from Create React App to Vite + TypeScript (CRA is unmaintained).
- Adopted Tailwind CSS, Redux Toolkit + RTK Query, and a feature-based folder structure.

### Added
- Login/signup pages (email+password and Google OAuth), request-robot-access flow, admin approve/reject panel, and a placeholder dashboard shell (Mapping/Navigation/Analytics/Documentation stubs).
- Route guards: `PublicRoute`, `ProtectedRoute`, `AdminRoute`, `RobotRequiredRoute`. Admins bypass the robot-assignment gate (`RobotRequiredRoute`) since they aren't expected to have a robot of their own.
- Silent access-token refresh on 401 via a custom RTK Query `baseQueryWithReauth`.
- `POST /api/auth/set-password` support (`SetPasswordForm`), surfaced on the dashboard home for accounts that signed up via Google only, so they can add a local password.
- Admin: "Add a robot" form + full robots list (all statuses, not just unassigned) on `AdminRequestsPage`, backed by `POST`/`GET /api/admin/robots`.
- Admin: new `/admin/users` page (`AdminUsersPage` + `UsersTable`) listing all users with a promote/demote control, backed by `GET /api/admin/users` and `PATCH /api/admin/users/:id/role`. Sidebar gets an "Admin: Users" link alongside "Admin: Requests".
- Forms (`LoginForm`, `SignupForm`, `SetPasswordForm`, `AddRobotForm`, the forgot-password wizard) migrated to React Hook Form + Zod (`@hookform/resolvers/zod`), replacing manual `useState` wiring.
- Vitest + React Testing Library test setup (`npm test`), with an initial `LoginForm` test suite.
- `/forgot-password`: a 3-step wizard (request OTP by email → verify 6-digit code → set new password), backed by `POST /api/auth/forgot-password`, `/verify-otp`, `/reset-password`. Linked from `LoginPage`.
- `PasswordInput`: shared show/hide password field (eye toggle icon), used by `LoginForm`, `SignupForm`, `SetPasswordForm`, and the reset step of the forgot-password wizard.
- Favicon/manifest assets (`favicon.ico`, `favicon-16/32.png`, `apple-touch-icon.png`, `icon-192/512.png`, `manifest.json`) generated from the AutoNav logo, closing the gap left when CRA's `public/` boilerplate was removed during the Vite migration. Sidebar now shows the icon next to the wordmark.
- Dark mode: Tailwind `darkMode: "class"` + a `ThemeProvider`/`useTheme` (state + `localStorage`, defaulting to system preference, applied pre-hydration in `index.html` to avoid a flash of the wrong theme). `ThemeToggle` (sun/moon button) added to the dashboard `Topbar` and to every pre-login page (login/signup/forgot-password/request-access). Dark variants added across all shared UI primitives and feature pages.

### Moved
- All pre-rewrite robot-control components (`MappingMap.js`, `JoystickView.js`, `NavigationPage.js`, `RobotControl.js`, etc.) moved verbatim into `src/legacy-robot-ui/`, disconnected from routing. Not ported or deleted — a future milestone decides their fate.
