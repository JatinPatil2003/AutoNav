# Changelog

All notable changes to the `AWS` deployment folder are documented here. Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- `nginx/dashboard.autonav.in.conf`: new site for the platform dashboard, HTTPS-only (port 80 redirects), reusing the existing wildcard `*.autonav.in` cert.
- `docker-compose.platform.yml`: `platform-mongo` (not exposed on a host port), `platform-backend` (`:4000`), `platform-frontend` (`:8081`) — bridge networking, no host networking/privileged mode.
- `.env.example` and `deploy.sh` for a scripted, idempotent EC2 deploy.
- `platform-backend` now bind-mounts `./logs/platform-backend` for its rotating `pino-roll` log files. `scripts/compress-old-logs.sh` (cron, not logrotate — see README) gzips rotated files and prunes `.gz` files older than 30 days.
