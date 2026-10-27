import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { viteSingleFile } from 'vite-plugin-singlefile'

// The output must stay a single self-contained ros2/panel.html: the panel
// node serves it from disk, with no asset directory next to it.
export default defineConfig({
  plugins: [react(), viteSingleFile()],
  build: { outDir: 'dist', emptyOutDir: true },
  test: {
    environment: 'happy-dom',
    globals: true,
    setupFiles: './tests/setup.js',
  },
})
