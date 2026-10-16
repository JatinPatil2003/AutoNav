# Development Mode — Laptop Reverse Proxy + Remote MongoDB

This describes a **temporary dev-mode wiring** of the `autonav.in` platform stack: `PlatformApp` and `PlatformServer` run on the developer's laptop (fast local iteration, no rebuild/redeploy to EC2 per change), while `platform-mongo` stays deployed on EC2 as the shared database. The public domains (`dashboard.autonav.in`, `platform-server.autonav.in`) keep working exactly as before — only *where the request ends up* changes.

It builds on two things already in place:
- The OpenVPN Access Server dynamic DNS setup (`docs/OpenVPN Access Server Dynamic Robot DNS Setup.md`): every connected VPN client gets a `<username>.robot` hostname that always resolves to its current VPN IP. The laptop's VPN client is named `dummy-server`, so from the EC2 host it's reachable at `dummy-server.robot`.
- The Cloudflare DNS zone for `autonav.in`, which now includes `platform-mongodb.autonav.in` as an **unproxied** (`cf_tags=cf-proxied:false`) A record pointing straight at the EC2 host — Cloudflare's proxy only understands HTTP(S), so the raw MongoDB wire protocol needs a direct, un-proxied record.

## Architecture

```text
Browser
  |
  | HTTPS (dashboard.autonav.in / platform-server.autonav.in)
  v
Cloudflare (proxied)
  |
  v
EC2 nginx  ──────────────────────────────┐
  |  (prod symlink: 127.0.0.1:8081/4000) │  (dev symlink: dummy-server.robot:5175/4000)
  v                                      v
platform-frontend / platform-backend     OpenVPN tunnel
containers (EC2, unused while in dev)         |
                                               v
                                   Laptop: PlatformApp (:5175)
                                           PlatformServer (:4000)
                                               |
                                               | MongoDB wire protocol
                                               v
                      platform-mongodb.autonav.in:27017 (unproxied DNS, direct to EC2)
                                               |
                                               v
                                   EC2: platform-mongo container
```

Only one of the two paths through nginx is active at a time, controlled by which file the `sites-enabled` symlink points at.

## 1. Mongo: published port

`AWS/docker-compose.platform.yml`'s `platform-mongo` service now publishes `27017:27017` (previously internal-only on the `platform-net` bridge network). Auth was already enforced via `MONGO_INITDB_ROOT_USERNAME` / `MONGO_INITDB_ROOT_PASSWORD` in `.env`, so the container itself doesn't become open-door just because the port is published — but the **EC2 security group** also needs an inbound rule for TCP 27017. This is a manual step in the AWS console (or CLI), not something `docker-compose.platform.yml` can express.

### Security group rule (manual, EC2 side)

Add one inbound rule to the security group attached to the EC2 instance:

| Field | Value |
| --- | --- |
| Type | Custom TCP |
| Protocol | TCP |
| Port range | `27017` |
| Source | your laptop's current public IP, as `/32` (e.g. `203.0.113.42/32`) — **not** `0.0.0.0/0` |
| Description | `platform-mongo dev access — <your name/laptop>` |

Find your current public IP with `curl -4 ifconfig.me` (run on the laptop, not the EC2 host). If your ISP rotates your IP, you'll need to update this rule when it changes — a tighter but more fragile trade-off than leaving it open. (A VPN-tunnel-only version of this, binding Mongo's published port to the EC2 host's VPN-side IP instead of all interfaces, is a future improvement **not** implemented here because the OpenVPN Access Server exposes several `as0tN` tunnel interfaces/subnets rather than one fixed address — see `docs/OpenVPN Access Server Dynamic Robot DNS Setup.md`.)

Console steps: **EC2 → Instances → (select instance) → Security tab → security group link → Edit inbound rules → Add rule** → fill in the table above → **Save rules**.

Equivalent AWS CLI (replace `sg-xxxxxxxx` and the IP):

```bash
aws ec2 authorize-security-group-ingress \
  --group-id sg-xxxxxxxx \
  --protocol tcp --port 27017 \
  --cidr 203.0.113.42/32
```

To remove the rule later (e.g. once dev mode ends), use `revoke-security-group-ingress` with the same arguments, or delete it from the console.

Redeploy with the normal flow:

```bash
cd AWS
docker compose -f docker-compose.platform.yml pull
./deploy.sh
```

### Connection string for the laptop

In the laptop's `PlatformServer/.env`, replace the local dev `MONGO_URI` with:

```
MONGO_URI=mongodb://platformadmin:<MONGO_INITDB_ROOT_PASSWORD>@platform-mongodb.autonav.in:27017/autonav_platform?authSource=admin
```

— using the real `MONGO_INITDB_ROOT_PASSWORD` value from the `.env` actually deployed on EC2 (the `AWS/.env.example` `changeme` is just a placeholder).

Verify connectivity from the laptop:

```bash
mongosh "mongodb://platformadmin:<password>@platform-mongodb.autonav.in:27017/autonav_platform?authSource=admin" --eval "db.runCommand({ping:1})"
```

## 2. nginx: swap EC2 to proxy toward the laptop

Two new conf files mirror the prod ones but proxy to `dummy-server.robot` over the VPN instead of `127.0.0.1`:

- `AWS/nginx/platform-server.autonav.in.dev.conf` → `dummy-server.robot:4000`
- `AWS/nginx/dashboard.autonav.in.dev.conf` → `dummy-server.robot:5175`

Both use an nginx `resolver` directive pointed at the local `systemd-resolved` stub (`127.0.0.53`) with a 5s TTL, and a `set $dev_upstream ...` variable, instead of a literal hostname in `proxy_pass`. This matters because `dummy-server.robot` is a *dynamic* record — the dnsmasq updater job re-writes it every ~10s from `VPNStatus`, so if the laptop reconnects and gets a new VPN IP, a literal hostname in `proxy_pass` would stay pinned to the old IP until the next `nginx reload`. The variable form re-resolves on every request instead.

One-time: copy the dev confs onto the host next to the existing ones.

```bash
sudo cp AWS/nginx/platform-server.autonav.in.dev.conf /etc/nginx/sites-available/platform-server.autonav.in.dev
sudo cp AWS/nginx/dashboard.autonav.in.dev.conf /etc/nginx/sites-available/dashboard.autonav.in.dev
```

**Switch to dev mode:**

```bash
sudo ln -sf /etc/nginx/sites-available/platform-server.autonav.in.dev /etc/nginx/sites-enabled/platform-server.autonav.in
sudo ln -sf /etc/nginx/sites-available/dashboard.autonav.in.dev /etc/nginx/sites-enabled/dashboard.autonav.in
sudo nginx -t && sudo systemctl reload nginx
```

**Switch back to prod mode** (point the same symlinks at the original files):

```bash
sudo ln -sf /etc/nginx/sites-available/platform-server.autonav.in /etc/nginx/sites-enabled/platform-server.autonav.in
sudo ln -sf /etc/nginx/sites-available/dashboard.autonav.in /etc/nginx/sites-enabled/dashboard.autonav.in
sudo nginx -t && sudo systemctl reload nginx
```

Don't forget to switch back once dev mode ends — otherwise the public domains keep pointing at a laptop that may be off or disconnected from the VPN.

## 3. Laptop-side app config

- `PlatformApp/vite.config.ts` sets `server.host = true` and `server.allowedHosts: ["dashboard.autonav.in"]`, since nginx forwards `Host: dashboard.autonav.in` and Vite's dev server otherwise binds to `localhost` only and rejects unrecognized `Host` headers. This only affects `vite dev` — the production container still serves a static build via its own nginx, so it's safe to leave in permanently.
- `PlatformServer` needs no code change — `server.listen(config.port, ...)` already binds to all interfaces, and its CORS allow-list already keys off `FRONTEND_URL` (`https://dashboard.autonav.in`), which doesn't change between prod and dev since that's the public-facing domain either way.

**Known limitation:** Vite HMR (hot module reload) over this proxy chain isn't specifically wired up — the dev server's HMR websocket assumes it's reachable at the same host/port the browser loaded the page from, which holds here, but TLS-terminated-at-nginx + VPN-tunnel hops add latency/complexity that hasn't been tested end-to-end. Plain page loads and manual reloads work; if HMR misbehaves, a manual browser refresh is the fallback.

## Verifying everything end-to-end

1. `docker compose -f AWS/docker-compose.platform.yml config` on EC2 — confirms the Mongo port mapping parses.
2. `mongosh` ping test above — confirms the laptop can reach `platform-mongo` directly.
3. `sudo nginx -t` after swapping symlinks — confirms the dev confs are valid.
4. Load `https://dashboard.autonav.in` in a browser — should render the laptop's Vite dev server.
5. Exercise an authenticated action on the dashboard — should hit `https://platform-server.autonav.in/api/...`, which should show up in the laptop's `PlatformServer` console/logs, and persist to `platform-mongo` on EC2 (check via `mongosh`).
