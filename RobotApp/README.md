# RobotApp

The local HMI (human-machine interface) PWA that runs on each robot's own touchscreen. This is where a human operator logs in with their account email + a robot-local 4-digit PIN — distinct from PlatformApp's email/password or Google login.

## Why this exists

RobotServer needs its own user logins and JWT-authenticated APIs (teleop, navigation, mapping, firmware), but PlatformApp (the cloud dashboard) never talks to a robot directly — PlatformServer is the only thing that reaches a robot's RobotServer, over the VPN. RobotApp is the thing that *does* talk to RobotServer directly, because it's served by that same RobotServer process, same-origin, on the robot itself.

## How login works

1. **Identify**: enter your account email. RobotServer checks its local roster cache (synced from PlatformServer) for whether you have access to this robot, and whether you've set a PIN here before.
2. **Known user, PIN already set**: enter your 4-digit PIN. RobotServer compares it against the locally cached hash — this works even if the robot's connection to PlatformServer is temporarily down.
3. **New user on this robot**: RobotServer relays an OTP request to PlatformServer (PlatformServer owns email delivery), you enter the code, then choose a 4-digit PIN. RobotServer caches the result and logs you in immediately.

See `RobotServer/src/modules/robot-auth/` for the server side of this flow, and `PlatformServer/src/modules/robot-auth/` for the service-to-service API it calls.

## Stack

Vite + React 18 + TypeScript + Tailwind, `vite-plugin-pwa` for installability on the robot's touchscreen. No Redux/RTK Query — this is a handful of screens, not a dashboard, so a plain `fetch` wrapper (`src/api/hmiClient.ts`) is enough. No React Router either — a small state machine in `App.tsx` covers identify → PIN login / PIN setup → logged in.

A few UI primitives (`src/components/ui/`) are intentionally duplicated from `PlatformApp` rather than shared via a package, to keep this app's build independent.

## Local development

```
npm install
npm run dev   # served at :5174, proxies /api to a locally running RobotServer on :8000
```

## Build & deploy

`npm run build` produces `dist/`, which `RobotServer/Dockerfile` copies into the robot's image and RobotServer serves as static files (same-origin, so RobotApp's session cookies need no CORS/SameSite handling).
