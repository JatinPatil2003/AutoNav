# ReactApp — AutoNav Operator Dashboard

The web operator dashboard: login, robot-assignment workflow, admin panel, and (in future milestones) mapping/navigation/analytics/documentation. Talks to `PlatformServer` for auth/users/robots, and — once a future milestone wires it up — to `WebServer` for live ROS telemetry/control.

Served exclusively over HTTPS in every deployed environment; see `AWS/nginx/dashboard.autonav.in.conf`.

## Stack

Vite, React 18, TypeScript, Tailwind CSS, Redux Toolkit + RTK Query, React Router.

This app was migrated off Create React App (CRA, now unmaintained) onto Vite as part of this rewrite.

## Architecture

Feature-based (`src/features/<feature>/{pages,components,api}`), with route guards in `src/router/` and shared UI primitives in `src/components/ui/`. Authentication state is **not** duplicated into a Redux slice — `useAuth()` (`src/features/auth/useAuth.ts`) derives it directly from RTK Query's cache for `GET /users/me`, so there's a single source of truth and nothing to desync after login/logout/refresh.

```
src/
  app/          Redux store, the RTK Query baseApi (with silent-refresh-on-401), error helpers
  router/       route tree + guards (PublicRoute, ProtectedRoute, AdminRoute, RobotRequiredRoute)
  features/     auth, robot-access, admin, dashboard (each: pages/ components/ *Api.ts)
  components/ui shared Tailwind-based primitives (Button, Input, Card, Spinner, PlaceholderPage)
  legacy-robot-ui/   pre-rewrite CRA components (MappingMap, JoystickView, etc.) — untouched,
                      not wired into routing; a future milestone decides whether to port or retire them
```

## Getting started

```bash
cp .env.example .env.local   # VITE_API_BASE_URL -> your local PlatformServer
npm install
npm run dev                  # http://localhost:5173
```

Run `PlatformServer` locally alongside this (see `../PlatformServer/README.md`) for the auth/API backend.

## Scripts

| Command | Purpose |
| --- | --- |
| `npm run dev` | Vite dev server with HMR |
| `npm run build` | Type-check + production build to `dist/` |
| `npm run preview` | Preview the production build locally |
| `npm run lint` | ESLint (excludes `src/legacy-robot-ui`) |

See the repo-root `CLAUDE.md` and `AWS/README.md` for how this is deployed to EC2.
