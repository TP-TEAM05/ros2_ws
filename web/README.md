# Web Application Directory

Place your web application source code in this directory.

## Supported Frameworks

### Docusaurus (Documentation Site)
```bash
cd web
npx create-docusaurus@latest . classic
```

Your structure will look like:
```
web/
├── docs/
├── src/
├── static/
├── docusaurus.config.js
├── package.json
└── ...
```

Build command: `npm run build` (outputs to `build/`)

### Quasar (Vue SPA Framework)
```bash
npm init quasar
# Follow the prompts
```

Your structure will look like:
```
web/
├── src/
├── public/
├── quasar.conf.js
├── package.json
└── ...
```

Build command: `npm run build` (outputs to `dist/spa/`)

### Other Static Site Generators

Any framework that produces static HTML/CSS/JS output will work. Just ensure:
1. Your `package.json` has a `build` script
2. The build output directory is correctly configured in `Dockerfile.web`

## Getting Started

1. Initialize your project in this directory
2. Adjust `Dockerfile.web` if needed (see comments for build output path)
3. Build and deploy using docker-compose:
   ```bash
   cd ..
   docker-compose -f docker-compose.web.yaml up -d --build web
   ```

## Configuration

Make sure your application is configured for production deployment:
- **Base URL**: Set to `/` (root)
- **Public Path**: Set to `/` 
- **Asset URLs**: Use relative paths

## Notes

This directory is gitignored by default. Add your application to version control by updating `.gitignore` if needed.
