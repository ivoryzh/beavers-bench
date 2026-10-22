import { defineConfig } from 'vite';

// The build goes to ../beavers_bench/static, which the Python package ships and the ivoryos-next plugin
// serves at /plugins/beavers_bench/. `base: './'` keeps every asset path relative, so the same build also opens
// from any static file server (sandbox mode) or a plain `file://` URL. On Vercel (which sets VERCEL=1, with
// the project's Root Directory set to frontend/) the same page is built into dist/, inside the project.
export default defineConfig({
  base: './',
  build: {
    outDir: process.env.VERCEL ? 'dist' : '../beavers_bench/static',
    emptyOutDir: true,
    assetsDir: 'assets',
  },
  test: {
    environment: 'node',
    include: ['tests/**/*.test.ts'],
  },
});
