# AWS

Everything deployed to the EC2 host for the AutoNav platform (auth/dashboard layer — `PlatformServer` + `PlatformApp`). This is separate from the robot-side `docker-compose.yml` at the repo root, which runs on the robot itself with host networking and device mounts.

The EC2 host already runs OpenVPN Access Server + nginx for `vpn.autonav.in`, `vpn-admin.autonav.in`, and `aldr-dashboard.autonav.in` (see `docs/OpenVPN Access Server + Nginx + Cloudflare Setup.md`). This folder adds a fourth site, `dashboard.autonav.in`, reusing the same wildcard `*.autonav.in` Cloudflare Origin certificate already installed at `/etc/nginx/ssl/autonav/`.

**HTTPS only.** Every request to `dashboard.autonav.in` is served over TLS; port 80 only exists to 301-redirect to HTTPS.

## Contents

| Path | Purpose |
| --- | --- |
| `nginx/dashboard.autonav.in.conf` | nginx server blocks (80→443 redirect, then path-based proxy to the frontend/backend containers) |
| `nginx/platform-server.autonav.in.dev.conf`, `nginx/dashboard.autonav.in.dev.conf` | dev-mode variants that proxy to the developer's laptop over the VPN instead of the local containers — see [Development mode](#development-mode) below |
| `docker-compose.platform.yml` | `platform-mongo`, `platform-backend`, `platform-frontend` services (bridge networking, explicit port mapping — no host networking/privileged mode, since these don't touch robot hardware) |
| `.env.example` | Env var template consumed by `docker-compose.platform.yml` |
| `deploy.sh` | `docker compose pull && up -d`, idempotent, run via SSH by CI or by hand |
| `scripts/compress-old-logs.sh` | Cron job: gzips `platform-backend`'s rotated log files, prunes `.gz` files older than 30 days |

## One-time EC2 setup (manual, not automated)

```bash
sudo cp AWS/nginx/dashboard.autonav.in.conf /etc/nginx/sites-available/dashboard.autonav.in
sudo ln -s /etc/nginx/sites-available/dashboard.autonav.in /etc/nginx/sites-enabled/
sudo nginx -t && sudo systemctl reload nginx
```

Also add a Cloudflare DNS record: `A dashboard → <EC2 IP>, Proxied` (matches the pattern of the existing `vpn`/`vpn-admin`/`aldr-dashboard` records).

## Deploying

```bash
cd AWS
cp .env.example .env   # fill in real secrets
docker compose -f docker-compose.platform.yml pull
./deploy.sh
```

In CI, this is automated by `.github/workflows/Platform_CI.yml`: it builds and pushes the `platform-backend`/`platform-frontend` images, copies this folder's compose file + a `.env` generated from GitHub secrets to EC2, and runs `deploy.sh` over SSH.

New GitHub repository secrets required (beyond the existing `DOCKER_USERNAME`/`DOCKER_PASSWORD`/`EC2_HOST`/`EC2_USERNAME`/`EC2_PRIVATE_KEY` used by `AutoNav_CI.yml`): `MONGO_ROOT_PW`, `JWT_ACCESS_SECRET`, `JWT_REFRESH_SECRET`, `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`, `ADMIN_EMAILS`.

## Logging

`platform-backend` writes structured logs via `pino`. In production it logs to both stdout (so `docker logs platform-backend` / any log-shipping agent on the host still works) and a rotating file under a bind-mounted `AWS/logs/platform-backend/` (daily or 20MB, whichever comes first, keeping the last 14 rotated files — see `PlatformServer/src/common/utils/logger.util.ts`).

`pino-roll` handles rotation and retention itself, but not compression. Add a daily cron entry on the EC2 host to gzip the rotated files it leaves behind:

```bash
crontab -e
# add:
0 3 * * * /home/<user>/AutoNav/AWS/scripts/compress-old-logs.sh
```

(Not a `logrotate` job on purpose — `pino-roll`'s active file is also named with a date/count pattern, so a `logrotate` glob can't reliably tell "active" from "already rotated away." The script instead only touches files untouched for 24h+, which `pino-roll` will have already rolled past by then.)

## Development mode

`platform-mongo`'s port (27017) is published so a laptop-hosted `PlatformServer` can connect to it directly at `platform-mongodb.autonav.in:27017`, and `nginx/*.dev.conf` let EC2 nginx reverse-proxy `dashboard.autonav.in`/`platform-server.autonav.in` to a laptop running `PlatformApp`/`PlatformServer` locally over the OpenVPN tunnel, instead of to the EC2 containers. See `docs/Development Mode - Laptop Reverse Proxy + Remote MongoDB.md` for the full setup, the Mongo connection string, and how to switch the nginx symlinks between dev and prod. Remember to switch back to the non-`.dev` confs when done.

## Why a separate Mongo from the robot's

The robot-side `autonav` MongoDB (maps/poses, via the repo-root `docker-compose.yml`) runs on the robot itself. `platform-mongo` here is a completely separate deployment on EC2 holding `autonav_platform` (users, robots, robot-access-requests) — different machine, different lifecycle, different concern. See `PlatformServer/README.md` for the schema.
