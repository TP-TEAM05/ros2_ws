# Web Deployment Guide for fiit-tp2025-team05.fiit.stuba.sk

This guide provides instructions for deploying the web application (Docusaurus/Quasar) on a Linux server with Docker and Nginx.

## Prerequisites

- Docker and Docker Compose installed on the Linux server
- Domain `fiit-tp2025-team05.fiit.stuba.sk` pointing to your server's IP address
- (Optional) SSL certificate for HTTPS

## Project Structure

```
.
├── Dockerfile.web          # Multi-stage Dockerfile for building and serving the web app
├── nginx.conf              # Main Nginx configuration
├── nginx-site.conf         # Site-specific Nginx configuration
├── docker-compose.web.yaml # Docker Compose configuration for web service
└── web/                    # Your web application source code directory
    ├── package.json
    ├── src/
    └── ...
```

## Setup Instructions

### 1. Prepare Your Web Application

Place your web application (Docusaurus or Quasar project) in a directory called `web/` in the repository root:

```bash
# For Docusaurus documentation site
mkdir -p web
cd web
npx create-docusaurus@latest . classic

# OR for Quasar application
npm init quasar
```

Ensure your `package.json` has the correct build script:
- **Docusaurus**: `"build": "docusaurus build"` (outputs to `build/`)
- **Quasar**: `"build": "quasar build"` (outputs to `dist/spa/`)

### 2. Adjust Dockerfile Based on Your Framework

#### For Docusaurus:
The default `Dockerfile.web` is configured for Docusaurus. No changes needed.

#### For Quasar:
Edit `Dockerfile.web` and change line 33-34:
```dockerfile
# Comment out:
# COPY --from=builder /app/build /usr/share/nginx/html

# Uncomment:
COPY --from=builder /app/dist/spa /usr/share/nginx/html
```

### 3. Build and Run with Docker Compose

```bash
# Build and start the web service
docker-compose -f docker-compose.web.yaml up -d web

# View logs
docker-compose -f docker-compose.web.yaml logs -f web

# Stop the service
docker-compose -f docker-compose.web.yaml down
```

### 4. Access Your Application

- **HTTP**: http://fiit-tp2025-team05.fiit.stuba.sk
- **Health Check**: http://fiit-tp2025-team05.fiit.stuba.sk/health

## SSL/HTTPS Configuration

### Using Let's Encrypt (Recommended)

1. Install Certbot on your Linux server:
```bash
sudo apt-get update
sudo apt-get install certbot python3-certbot-nginx
```

2. Obtain SSL certificate:
```bash
sudo certbot certonly --nginx -d fiit-tp2025-team05.fiit.stuba.sk
```

3. Create SSL directory and copy certificates:
```bash
mkdir -p ssl
sudo cp /etc/letsencrypt/live/fiit-tp2025-team05.fiit.stuba.sk/fullchain.pem ./ssl/cert.pem
sudo cp /etc/letsencrypt/live/fiit-tp2025-team05.fiit.stuba.sk/privkey.pem ./ssl/key.pem
sudo chmod 644 ./ssl/*.pem
```

4. Uncomment HTTPS section in `nginx-site.conf`:
   - Uncomment the HTTPS server block (lines starting with `# server {` for port 443)
   - Uncomment the HTTP to HTTPS redirect block at the bottom

5. Update `docker-compose.web.yaml` to mount SSL certificates:
```yaml
volumes:
  - ./ssl:/etc/nginx/ssl:ro
```

6. Rebuild and restart:
```bash
docker-compose -f docker-compose.web.yaml up -d --build web
```

### Using Custom SSL Certificates

If you have your own SSL certificates:

1. Place certificates in the `ssl/` directory:
   - `ssl/cert.pem` - Full certificate chain
   - `ssl/key.pem` - Private key

2. Follow steps 4-6 from the Let's Encrypt section above.

## Alternative: Using Nginx Directly on Host (Without Docker)

If you prefer running Nginx directly on the host system:

1. Build the web application:
```bash
cd web
npm install
npm run build
```

2. Install Nginx:
```bash
sudo apt-get update
sudo apt-get install nginx
```

3. Copy Nginx configuration:
```bash
sudo cp nginx-site.conf /etc/nginx/sites-available/fiit-tp2025-team05
sudo ln -s /etc/nginx/sites-available/fiit-tp2025-team05 /etc/nginx/sites-enabled/
```

4. Copy built files to web root:
```bash
sudo mkdir -p /var/www/fiit-tp2025-team05
# For Docusaurus:
sudo cp -r web/build/* /var/www/fiit-tp2025-team05/
# For Quasar:
# sudo cp -r web/dist/spa/* /var/www/fiit-tp2025-team05/
```

5. Update `nginx-site.conf` root path:
```nginx
root /var/www/fiit-tp2025-team05;
```

6. Test and reload Nginx:
```bash
sudo nginx -t
sudo systemctl reload nginx
```

## Troubleshooting

### Port Already in Use
If port 80 or 443 is already in use:
```bash
# Check what's using the ports
sudo lsof -i :80
sudo lsof -i :443

# Stop conflicting services
sudo systemctl stop apache2  # if Apache is running
```

### Permission Issues
```bash
# Ensure proper permissions for SSL certificates
sudo chmod 644 ssl/cert.pem
sudo chmod 600 ssl/key.pem
```

### Container Not Starting
```bash
# Check container logs
docker-compose -f docker-compose.web.yaml logs web

# Check Nginx configuration syntax
docker-compose -f docker-compose.web.yaml exec web nginx -t
```

### Static Assets Not Loading
Ensure your web application's `publicPath` or `baseUrl` is configured correctly:
- **Docusaurus**: Set `baseUrl: '/'` in `docusaurus.config.js`
- **Quasar**: Set `publicPath: '/'` in `quasar.conf.js`

## Production Deployment Checklist

- [ ] Domain DNS configured to point to server IP
- [ ] Firewall rules allow ports 80 and 443
- [ ] SSL certificates installed and configured
- [ ] Nginx security headers enabled (in nginx-site.conf)
- [ ] Nginx gzip compression enabled (in nginx.conf)
- [ ] Application builds successfully
- [ ] Health check endpoint returns 200 OK
- [ ] All routes accessible (test SPA routing)
- [ ] Static assets cached properly
- [ ] Logs directory created and writable
- [ ] Automatic certificate renewal configured (for Let's Encrypt)

## Maintenance

### Updating the Application

```bash
# Pull latest changes
git pull origin main

# Rebuild and restart
docker-compose -f docker-compose.web.yaml up -d --build web
```

### Renewing SSL Certificates (Let's Encrypt)

Certbot automatically renews certificates. To test renewal:
```bash
sudo certbot renew --dry-run
```

### Viewing Logs

```bash
# Application logs
docker-compose -f docker-compose.web.yaml logs -f web

# Nginx access logs
tail -f logs/nginx/access.log

# Nginx error logs
tail -f logs/nginx/error.log
```

## Support

For issues specific to:
- **Docusaurus**: https://docusaurus.io/docs
- **Quasar**: https://quasar.dev/
- **Nginx**: https://nginx.org/en/docs/
- **Docker**: https://docs.docker.com/

## Notes

- The current configuration uses HTTP (port 80). For production, always use HTTPS.
- The Dockerfile uses multi-stage builds to create a lightweight production image.
- Nginx is configured to handle both static sites and SPA routing.
- Adjust `client_max_body_size` in `nginx.conf` if you need to upload larger files.
