import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { viteSingleFile } from 'vite-plugin-singlefile'

// The Rust server serves a single self-contained web/panel.html from disk,
// with no asset directory next to it.
export default defineConfig({
  plugins: [react(), viteSingleFile(), {
    name: 'panel-html',
    enforce: 'post',
    generateBundle: {
      order: 'post',
      handler(_, bundle) {
        const html = bundle['index.html']
        html.fileName = 'panel.html'
        bundle['panel.html'] = html
        delete bundle['index.html']
      },
    },
  }],
  build: { outDir: '../../web', emptyOutDir: false },
  test: {
    environment: 'happy-dom',
    globals: true,
    setupFiles: './tests/setup.js',
  },
})
