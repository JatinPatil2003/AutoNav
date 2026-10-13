#!/usr/bin/env bash
# Deploys/updates the platform stack (mongo, backend, frontend) on the EC2
# host. Idempotent — safe to re-run. Expects docker-compose.platform.yml and
# .env to already be present in this directory (CI copies them here before
# running this script over SSH; see .github/workflows/Platform_CI.yml).
set -euo pipefail
cd "$(dirname "$0")"

docker compose -f docker-compose.platform.yml pull
docker compose -f docker-compose.platform.yml up -d --remove-orphans
docker image prune -f
