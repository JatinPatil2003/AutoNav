# PlatformServer

The AutoNav platform backend — authentication, user management, and the robot-assignment workflow for the operator dashboard. This is a separate concern from `WebServer` (the FastAPI ROS bridge): `PlatformServer` owns accounts, roles, and which user is assigned which robot; `WebServer` owns live ROS telemetry/control for a given robot.

Communication with this API happens over HTTPS only in every deployed environment (see `AWS/nginx/dashboard.autonav.in.conf`); plain HTTP is only ever used on `localhost` during local development.

## Stack

Node.js, Express, TypeScript, Mongoose (MongoDB), Passport (Google OAuth2), JWT access/refresh tokens in httpOnly cookies, Zod for validation, Jest for tests, OpenAPI docs via `swagger-jsdoc`/`swagger-ui-express`.

## Architecture

Layered, interface-first: `routes → controller → service → repository (interface) → Mongoose model`. Every service depends on a repository **interface** (e.g. `IUserRepository`), not a concrete Mongoose implementation, which is what lets the unit tests swap in in-memory fakes (`tests/unit/fakes/`) instead of needing a real database.

```
src/
  config/      env validation (zod), Passport strategy, Swagger spec
  db/          Mongoose connection
  common/      cross-cutting middleware, errors, utils, shared types
  modules/     auth, users, robots, robot-access-requests, admin
```

Composition (dependency wiring) happens in `src/app.ts` — the single place that instantiates repositories, services, and controllers and wires them into Express routers.

## Getting started

```bash
cp .env.example .env     # fill in real secrets; see comments in .env.example
docker run -d -p 27017:27017 --name dev-mongo mongo:6.0
npm install
npm run dev              # nodemon + ts-node, http://localhost:4000
```

Google OAuth requires real `GOOGLE_CLIENT_ID`/`GOOGLE_CLIENT_SECRET`/`GOOGLE_CALLBACK_URL` values from the Google Cloud Console; the email/password flow works with placeholder values.

## Scripts

| Command | Purpose |
| --- | --- |
| `npm run dev` | Run with hot reload |
| `npm run build` | Compile TypeScript to `dist/` |
| `npm start` | Run the compiled build |
| `npm run lint` | ESLint |
| `npm test` | Jest (unit + integration, the latter via `mongodb-memory-server`) |
| `npm run seed:robot -- --name=autonav --hostname=autonav.robot` | Provision a `Robot` record for admins to assign |

## API docs

Once running, interactive OpenAPI docs are served at `GET /api/docs`. The spec is generated from `@openapi` JSDoc annotations on each `*.routes.ts` file, so it can't drift out of sync with the route definitions silently.

## First admin

Add your email to `ADMIN_EMAILS` (comma-separated) before your first signup/Google login — the role is resolved from that list only at account-creation time.

See the repo-root `CLAUDE.md` and `AWS/README.md` for how this service is deployed to EC2.
