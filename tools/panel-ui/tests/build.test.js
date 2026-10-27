// FEATURE: The Rust HTTP server serves one self-contained web/panel.html.
// SPEC: Build into web without deleting unrelated files; rename inlined HTML.
// IMPLEMENTATION: Rename the single-file output after inlining; inline the favicon.
// RESULT: IMPLEMENTED — the bundle needs no separate static assets.
// STATUS: DONE
import { expect, it } from 'vitest'
import { resolve } from 'node:path'
import { readFileSync } from 'node:fs'
import config from '../vite.config.js'

it('does not request a separate favicon asset', () => {
  const html = readFileSync(resolve(import.meta.dirname, '../index.html'), 'utf8')
  expect(html).not.toMatch(/href="\/(?!\/)/)
})

it('emits panel.html under web instead of the ROS directory', () => {
  expect(resolve(import.meta.dirname, '..', config.build.outDir)).toBe(resolve(import.meta.dirname, '../../../web'))
  expect(config.build.emptyOutDir).toBe(false)
  const plugin = config.plugins.flat().find((p) => p.name === 'panel-html')
  expect(plugin).toBeTruthy()
  const bundle = { 'index.html': { type: 'asset', fileName: 'index.html', source: '<html>inlined</html>' } }
  plugin.generateBundle.handler({}, bundle)
  expect(Object.keys(bundle)).toEqual(['panel.html'])
  expect(bundle['panel.html'].fileName).toBe('panel.html')
  expect(bundle['panel.html'].source).toBe('<html>inlined</html>')
})
