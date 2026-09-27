# OpenVPN Access Server + Nginx + Cloudflare Setup

This document describes the setup completed for `autonav.in`, including:

* Cloudflare DNS configuration
* OpenVPN Access Server port changes
* Nginx reverse proxy
* Cloudflare Origin SSL certificate
* OpenVPN Admin Portal
* OpenVPN Client Portal
* ALDR Dashboard

---

## 1. Domain & Cloudflare DNS

Domain:

```text
autonav.in
```

Cloudflare nameservers configured at GoDaddy:

```text
arnold.ns.cloudflare.com
melina.ns.cloudflare.com
```

### DNS Records

Create the following DNS records in Cloudflare:

| Type | Name             | Target         | Proxy   |
| ---- | ---------------- | -------------- | ------- |
| A    | `vpn`            | `13.232.77.10` | Proxied |
| A    | `vpn-admin`      | `13.232.77.10` | Proxied |
| A    | `aldr-dashboard` | `13.232.77.10` | Proxied |

Resulting domains:

```text
vpn.autonav.in
vpn-admin.autonav.in
aldr-dashboard.autonav.in
```

---

# 2. OpenVPN Access Server Ports

OpenVPN Access Server was configured so that Nginx can use HTTPS port `443`.

## Change OpenVPN TCP port

```bash
sudo /usr/local/openvpn_as/scripts/sacli \
  --key "vpn.server.daemon.tcp.port" \
  --value "8443" \
  ConfigPut
```

## Change OpenVPN UDP port

```bash
sudo /usr/local/openvpn_as/scripts/sacli \
  --key "vpn.server.daemon.udp.port" \
  --value "1194" \
  ConfigPut
```

## Change OpenVPN daemon listener

```bash
sudo /usr/local/openvpn_as/scripts/sacli \
  --key "vpn.daemon.0.listen.port" \
  --value "8443" \
  ConfigPut
```

## Restart OpenVPN Access Server

```bash
sudo /usr/local/openvpn_as/scripts/sacli start
```

OpenVPN Access Server ports:

```text
TCP 8443
UDP 1194
HTTPS Admin Portal: 943
```

---

# 3. Cloudflare Origin Certificate

Create a Cloudflare Origin Certificate for:

```text
*.autonav.in
```

Store the certificate and private key:

```text
/etc/nginx/ssl/autonav/fullchain.pem
/etc/nginx/ssl/autonav/privkey.pem
```

Create the directory:

```bash
sudo mkdir -p /etc/nginx/ssl/autonav
```

Set appropriate permissions:

```bash
sudo chmod 600 /etc/nginx/ssl/autonav/privkey.pem
sudo chmod 644 /etc/nginx/ssl/autonav/fullchain.pem
```

Cloudflare SSL/TLS mode:

```text
Full (strict)
```

---

# 4. Nginx Installation

Install Nginx:

```bash
sudo apt update
sudo apt install nginx -y
```

Enable and start Nginx:

```bash
sudo systemctl enable nginx
sudo systemctl start nginx
```

---

# 5. Nginx Configuration

Create the site configuration:

```bash
sudo nano /etc/nginx/sites-available/autonav
```

Configuration:

```nginx
server {
    listen 443 ssl;
    server_name vpn.autonav.in;

    ssl_certificate     /etc/nginx/ssl/autonav/fullchain.pem;
    ssl_certificate_key /etc/nginx/ssl/autonav/privkey.pem;

    location / {
        proxy_pass https://13.232.77.10:8443;

        proxy_ssl_verify off;

        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto https;
        proxy_set_header X-Forwarded-Host $host;
    }
}


server {
    listen 443 ssl;
    server_name vpn-admin.autonav.in;

    ssl_certificate     /etc/nginx/ssl/autonav/fullchain.pem;
    ssl_certificate_key /etc/nginx/ssl/autonav/privkey.pem;

    location = / {
        return 302 /admin/login;
    }

    location / {
        proxy_pass https://127.0.0.1:943;

        proxy_ssl_verify off;

        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto https;
        proxy_set_header X-Forwarded-Host $host;
    }
}


server {
    listen 443 ssl;
    server_name aldr-dashboard.autonav.in;

    ssl_certificate     /etc/nginx/ssl/autonav/fullchain.pem;
    ssl_certificate_key /etc/nginx/ssl/autonav/privkey.pem;

    location / {
        proxy_pass http://127.0.0.1:5654;

        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto https;
        proxy_set_header X-Forwarded-Host $host;

        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

---

# 6. Enable Nginx Site

Create the symbolic link:

```bash
sudo ln -s /etc/nginx/sites-available/autonav \
    /etc/nginx/sites-enabled/autonav
```

Remove the default Nginx site:

```bash
sudo rm -f /etc/nginx/sites-enabled/default
```

Reload Nginx:

```bash
sudo systemctl reload nginx
```

---

# 7. Final Architecture

```text
                         Cloudflare
                             │
                             │ HTTPS :443
                             ▼
                     ┌─────────────────┐
                     │      Nginx      │
                     │      :443       │
                     └────────┬────────┘
                              │
             ┌────────────────┼────────────────┐
             │                │                │
             ▼                ▼                ▼
     vpn.autonav.in   vpn-admin.autonav.in   aldr-dashboard.autonav.in
             │                │                │
             ▼                ▼                ▼
      OpenVPN AS :8443  OpenVPN AS :943     Dashboard :5654
```

## Domain Mapping

### OpenVPN Client Portal

```text
https://vpn.autonav.in
        │
        ▼
https://13.232.77.10:8443
```

### OpenVPN Admin Portal

```text
https://vpn-admin.autonav.in
        │
        ▼
/admin/login
        │
        ▼
https://127.0.0.1:943/admin/login
```

Therefore:

```text
https://vpn-admin.autonav.in
```

automatically redirects to:

```text
https://vpn-admin.autonav.in/admin/login
```

### ALDR Dashboard

```text
https://aldr-dashboard.autonav.in
        │
        ▼
http://127.0.0.1:5654
```

---

# 8. Ports Summary

| Service          |   Port | Exposure      |
| ---------------- | -----: | ------------- |
| Nginx HTTPS      |  `443` | Public        |
| OpenVPN TCP      | `8443` | Public        |
| OpenVPN UDP      | `1194` | Public        |
| OpenVPN Admin UI |  `943` | Through Nginx |
| ALDR Dashboard   | `5654` | Through Nginx |

The public web services are exposed through Nginx on port `443`, while the backend services remain on their respective ports.
