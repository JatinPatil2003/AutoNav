#!/usr/bin/env bash
# Gzips rotated PlatformServer log files that pino-roll (inside the
# platform-backend container) is no longer writing to, and deletes gzipped
# logs older than 30 days.
#
# Not a logrotate job on purpose: pino-roll's *active* file is also named
# with a date/count pattern (not a fixed "app.log"), so a logrotate glob
# can't safely tell active from rotated-away. Only gzipping files that
# haven't been modified in the last day sidesteps that race entirely —
# pino-roll will have already rolled onto a new file by then.
#
# Run via cron on the EC2 host, e.g. a daily entry:
#   0 3 * * * /home/<user>/AutoNav/AWS/scripts/compress-old-logs.sh
set -euo pipefail

LOG_DIR="$(dirname "$0")/../logs/platform-backend"

if [ ! -d "$LOG_DIR" ]; then
  exit 0
fi

find "$LOG_DIR" -name 'app.*.log' -mmin +1440 -exec gzip {} \;
find "$LOG_DIR" -name '*.log.gz' -mtime +30 -delete
