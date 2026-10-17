# OpenVPN Access Server — Dynamic Robot DNS

This setup provides dynamic DNS names for OpenVPN clients.

Example:

```text
autonav.robot  → current VPN IP of user autonav
openvpn.robot  → current VPN IP of user openvpn
```

The IP addresses are obtained dynamically from OpenVPN Access Server using `VPNStatus`. Nothing is hardcoded for the robot VPN IPs.

## Architecture

```text
OpenVPN Client
      |
      | DNS
      v
Access Server dnsmasq
      |
      +-- *.robot → /etc/dnsmasq-robots.hosts
      |
      +-- Internet DNS → 172.31.0.2
```

Current VPN networks:

```text
172.27.224.0/22
172.27.228.0/22
172.27.232.0/22
172.27.236.0/22
```

DNS servers provided by dnsmasq:

```text
172.27.224.1
172.27.228.1
172.27.232.1
172.27.236.1
```

---

# 1. Install dnsmasq

Run on the OpenVPN Access Server:

```bash
sudo apt update
sudo apt install -y dnsmasq
```

---

# 2. Configure dnsmasq

Create:

```bash
sudo tee /etc/dnsmasq.d/openvpn-robots.conf > /dev/null <<'EOF'
# Robot VPN DNS

domain=robot
local=/robot/

interface=as0t0
interface=as0t1
interface=as0t2
interface=as0t3

bind-interfaces

# Upstream DNS
server=172.31.0.2

# Dynamically generated robot records
addn-hosts=/etc/dnsmasq-robots.hosts
EOF
```

---

# 3. Create the dynamic hosts file

```bash
sudo touch /etc/dnsmasq-robots.hosts
sudo chmod 644 /etc/dnsmasq-robots.hosts
```

---

# 4. Create the dynamic DNS updater

Create:

```bash
sudo tee /usr/local/bin/update-robot-dns.sh > /dev/null <<'EOF'
#!/bin/bash

HOSTS="/etc/dnsmasq-robots.hosts"
TMP="/tmp/dnsmasq-robots.hosts"

VPNSTATUS="/usr/local/openvpn_as/scripts/sacli VPNStatus"

$VPNSTATUS | python3 -c '
import sys
import json

data = json.load(sys.stdin)

for daemon in data.values():
    for client in daemon.get("client_list", []):
        header = daemon.get("client_list_header", {})

        username = client[header["Username"]]
        common_name = client[header["Common Name"]]
        virtual_ip = client[header["Virtual Address"]]

        hostname = username or common_name

        if hostname and virtual_ip:
            print(f"{virtual_ip} {hostname}.robot")
' | sort -k2 > "$TMP"

if ! cmp -s "$TMP" "$HOSTS"; then
    mv "$TMP" "$HOSTS"
    chmod 644 "$HOSTS"

    if systemctl is-active --quiet dnsmasq; then
        kill -HUP "$(pidof dnsmasq)"
    fi
else
    rm -f "$TMP"
fi
EOF
```

Make it executable:

```bash
sudo chmod +x /usr/local/bin/update-robot-dns.sh
```

---

# 5. Start dnsmasq

```bash
sudo systemctl enable dnsmasq
sudo systemctl restart dnsmasq
```

---

# 6. Create automatic DNS updater service

```bash
sudo tee /etc/systemd/system/robot-dns-update.service > /dev/null <<'EOF'
[Unit]
Description=Update robot DNS records from OpenVPN Access Server
After=openvpnas.service dnsmasq.service
Wants=dnsmasq.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/update-robot-dns.sh
EOF
```

---

# 7. Create automatic DNS updater timer

```bash
sudo tee /etc/systemd/system/robot-dns-update.timer > /dev/null <<'EOF'
[Unit]
Description=Continuously update robot DNS records

[Timer]
OnBootSec=5s
OnUnitActiveSec=10s
Unit=robot-dns-update.service

[Install]
WantedBy=timers.target
EOF
```

---

# 8. Enable the automatic updater

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot-dns-update.timer
```

The updater runs every 10 seconds.

---

# 9. Configure OpenVPN Access Server DNS

Set Access Server to use custom DNS:

```bash
sudo /usr/local/openvpn_as/scripts/sacli \
  --key "vpn.client.routing.reroute_dns" \
  --value "custom" ConfigPut
```

Set the DNS server:

```bash
sudo /usr/local/openvpn_as/scripts/sacli \
  --key "vpn.server.dhcp_option.dns.0" \
  --value "172.27.224.1" ConfigPut
```

Apply the configuration:

```bash
sudo /usr/local/openvpn_as/scripts/sacli Start
```

---

# 10. Configure systemd-resolved for Robot DNS

The OpenVPN Access Server uses `systemd-resolved`, while dnsmasq serves the `.robot` domain.

Route `.robot` queries to dnsmasq **globally**, not per-link — a per-link routing domain (`resolvectl dns/domain as0t0 ...`) only applies to queries already associated with that one tunnel interface, and a client can land on any of the four `as0tN` adapters/subnets depending on which one OpenVPN Access Server happens to assign it. A global drop-in avoids that: it routes any `*.robot` query to dnsmasq regardless of which adapter the client ended up on, trying each of the four per-subnet dnsmasq-serving addresses in turn.

```bash
sudo mkdir -p /etc/systemd/resolved.conf.d
sudo tee /etc/systemd/resolved.conf.d/90-robot.conf > /dev/null <<'EOF'
[Resolve]
DNS=172.27.224.1 172.27.228.1 172.27.232.1 172.27.236.1
Domains=~robot
EOF
sudo systemctl restart systemd-resolved
```

(An earlier version of this doc configured this per-link on `as0t0` only via a `robot-dns-resolved.service` unit — that's superseded by the drop-in above. If that unit still exists on your host, disable it so the two configs don't fight each other: `sudo systemctl disable --now robot-dns-resolved.service`.)

DNS routing:

```text
*.robot
   |
   v
172.27.224.1
   |
   v
dnsmasq
   |
   v
/etc/dnsmasq-robots.hosts
```

Normal DNS:

```text
Internet DNS
   |
   v
172.31.0.2
```

---

# 11. Reconnect OpenVPN clients

Reconnect all OpenVPN clients so they receive the new DNS configuration.

For the `autonav` robot using systemd:

```bash
sudo systemctl restart openvpn-client@autonavvpn.service
```

---

# 12. DNS behavior

Robot hostname:

```text
autonav.robot
```

is resolved from:

```text
/etc/dnsmasq-robots.hosts
```

Normal Internet DNS is forwarded to:

```text
172.31.0.2
```

Therefore:

```text
autonav.robot → dynamic VPN IP
openvpn.robot → dynamic VPN IP
google.com    → 172.31.0.2
```

---

# 13. Automatic IP updates

The systemd timer runs every 10 seconds:

```text
Every 10 seconds
       |
       v
OpenVPN Access Server VPNStatus
       |
       v
Current connected users
       |
       v
Current VPN IP addresses
       |
       v
/etc/dnsmasq-robots.hosts
       |
       v
dnsmasq
```

If a robot reconnects and receives a different VPN IP, its `.robot` DNS record is automatically updated.

No robot VPN IP needs to be manually configured.

---

# 14. Important files

```text
/etc/dnsmasq.d/openvpn-robots.conf
/etc/dnsmasq-robots.hosts
/usr/local/bin/update-robot-dns.sh
/etc/systemd/system/robot-dns-update.service
/etc/systemd/system/robot-dns-update.timer
/etc/systemd/system/robot-dns-resolved.service
```

---

# 15. Client usage

Once connected to the VPN:

```bash
ping autonav.robot
```

or:

```bash
ssh user@autonav.robot
```

The hostname resolves to the robot's current VPN address.

---

# 16. Static DNS for `platform.internal` (reverse direction: robot → PlatformServer)

Everything above resolves a *robot's* dynamic VPN address so the EC2 host (or another peer) can reach it. The reverse direction needs the opposite kind of record: `RobotServer` (running on the robot) connects out to `PlatformServer` at `platform.internal:4000` directly over the VPN tunnel — bypassing nginx/Cloudflare/TLS entirely. This record remains useful as a VPN-only fallback path, but it is no longer the trust boundary for `/ws/robot-link`: that path is now also proxied publicly over HTTPS at `https://platform-server.autonav.in/ws/robot-link` (see `AWS/nginx/platform-server.autonav.in.conf`), authenticated by `ROBOT_LINK_SECRET` (see the comments in `PlatformServer/src/index.ts`) rather than VPN membership. Unlike a robot's address, EC2's own VPN-side address doesn't roam — so this is a **static** record, not a dynamically-updated one, and it must live in a file the `update-robot-dns.sh` timer never touches (that script overwrites `/etc/dnsmasq-robots.hosts` wholesale every 10s).

Create a separate static hosts file:

```bash
echo "172.27.224.1 platform.internal" | sudo tee /etc/dnsmasq-platform.hosts
sudo chmod 644 /etc/dnsmasq-platform.hosts
```

`172.27.224.1` is one of the four dnsmasq-serving gateway addresses from step "DNS servers provided by dnsmasq" above — any of them reaches EC2 itself (the OpenVPN Access Server), regardless of which `as0tN` subnet a given robot happens to be assigned to, since EC2 is the server all four belong to.

Add a second `addn-hosts` line to the dnsmasq config from step 2 (dnsmasq supports multiple `addn-hosts` directives, merging all of them):

```bash
sudo sed -i '/^addn-hosts=\/etc\/dnsmasq-robots.hosts$/a addn-hosts=/etc/dnsmasq-platform.hosts' /etc/dnsmasq.d/openvpn-robots.conf
```

Restart dnsmasq to pick it up:

```bash
sudo systemctl restart dnsmasq
```

Verify from a connected robot:

```bash
ping platform.internal
```

If `platform.internal`'s actual host ever changes (e.g. EC2 instance replaced, or PlatformServer moved to different infrastructure), update the IP in `/etc/dnsmasq-platform.hosts` and `sudo systemctl restart dnsmasq` — nothing else needs to change, since `RobotServer/.env.example`'s `PLATFORM_BASE_URL=http://platform.internal:4000` stays the same either way.
