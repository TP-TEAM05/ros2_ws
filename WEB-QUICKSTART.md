# Quick Start - Web Deployment

Deploy your web application to `fiit-tp2025-team05.fiit.stuba.sk`

## Quick Setup

1. **Place your web app in the `web/` directory**
   ```bash
   mkdir -p web
   # Copy your Docusaurus or Quasar project here
   ```

2. **Build and run**
   ```bash
   docker-compose -f docker-compose.web.yaml up -d --build web
   ```

3. **Access your site**
   - HTTP: http://fiit-tp2025-team05.fiit.stuba.sk
   - Health: http://fiit-tp2025-team05.fiit.stuba.sk/health

## For Production

1. **Get SSL Certificate**
   ```bash
   sudo certbot certonly --nginx -d fiit-tp2025-team05.fiit.stuba.sk
   mkdir -p ssl
   sudo cp /etc/letsencrypt/live/fiit-tp2025-team05.fiit.stuba.sk/fullchain.pem ./ssl/cert.pem
   sudo cp /etc/letsencrypt/live/fiit-tp2025-team05.fiit.stuba.sk/privkey.pem ./ssl/key.pem
   ```

2. **Enable HTTPS in nginx-site.conf**
   - Uncomment HTTPS server block
   - Uncomment HTTP→HTTPS redirect

3. **Update docker-compose.web.yaml**
   ```yaml
   volumes:
     - ./ssl:/etc/nginx/ssl:ro
   ```

4. **Rebuild**
   ```bash
   docker-compose -f docker-compose.web.yaml up -d --build web
   ```

## Complete Guide

See [DEPLOYMENT.md](DEPLOYMENT.md) for detailed instructions.
