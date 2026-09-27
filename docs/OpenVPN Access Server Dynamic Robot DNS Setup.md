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

Configure `systemd-resolved` to route only `.robot` queries to dnsmasq:

```bash
sudo resolvectl dns as0t0 172.27.224.1
sudo resolvectl domain as0t0 '~robot'
```

Create a persistent systemd service:

```bash
sudo tee /etc/systemd/system/robot-dns-resolved.service > /dev/null <<'EOF'
[Unit]
Description=Configure systemd-resolved for OpenVPN robot DNS
After=systemd-resolved.service openvpnas.service
Wants=systemd-resolved.service

[Service]
Type=oneshot
ExecStart=/usr/bin/resolvectl dns as0t0 172.27.224.1
ExecStart=/usr/bin/resolvectl domain as0t0 ~robot
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
```

Enable the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot-dns-resolved.service
```

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
