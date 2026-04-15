import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// When building for GitHub Pages the site is served from
//   https://<user>.github.io/<repo>/
// so all asset URLs need to be prefixed with the repo name. Local dev and
// other deployments use the root path.
const isGithubPages = process.env.GITHUB_PAGES === 'true';

export default defineConfig({
  base: isGithubPages ? '/climb_sim/' : '/',
  plugins: [react()],
  server: {
    port: 5173,
    host: true,
  },
});
