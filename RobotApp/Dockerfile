# Built with the repo root as context, matching RobotServer/Dockerfile and
# docker-compose.yml's `context: .` / `dockerfile: RobotApp/Dockerfile` — so
# every path below is root-relative.

FROM node:20-slim AS build

WORKDIR /robotapp
COPY RobotApp/package*.json ./
RUN npm ci

COPY RobotApp/. .
RUN npm run build

# Nginx serves the static PWA build and reverse-proxies /api, /ws, /maps,
# /health to RobotServer on localhost:8000 (same host, network_mode: host —
# see docker-compose.yml) so the browser sees one origin and RobotServer's
# session cookie needs no CORS/SameSite handling.
FROM nginx:1.27-alpine

COPY --from=build /robotapp/dist /usr/share/nginx/html
COPY RobotApp/nginx.conf /etc/nginx/nginx.conf

EXPOSE 80
