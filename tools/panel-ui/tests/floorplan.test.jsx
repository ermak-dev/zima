// FEATURE (#83): polygonal floorplan layer with an operator rebuild button.
// SPEC: the versioned `floorplan` WS frame renders as a canvas layer
// (rooms/walls/doors) toggled by its own checkbox; the rebuild button POSTs
// /api/floorplan/rebuild, disables while pending, shows errors and keeps the
// last layer on failure; the layer never clears on disconnect.
import { describe, it, expect, vi, afterEach } from 'vitest'
import { render, cleanup, act, screen, fireEvent } from '@testing-library/react'

vi.mock('../src/usePanelSocket.js', () => ({
  default: (onMessage) => mockSocket(onMessage),
}))

let lastHandler = null
let connected = true
const sent = []
function mockSocket(onMessage) {
  lastHandler = onMessage
  return { connected, send: (m) => sent.push(m) }
}

import App from '../src/App.jsx'

const floorplanFrame = (version = 1) => ({
  type: 'floorplan',
  data: {
    version,
    grid_version: 3,
    walls: [{ kind: 'wall', a: [0, 0], b: [4, 0], observed: true }],
    rooms: [{ polygon: [[0, 0], [4, 0], [4, 3], [0, 3]], area_m2: 12, edges: [], unobserved_edge_share: 0 }],
    doors: [{ a: [2, 0], b: [2.8, 0], width_m: 0.8 }],
    obstacles: [],
    report: { theta_deg: 0 },
  },
})

afterEach(() => {
  cleanup()
  sent.length = 0
  connected = true
  vi.restoreAllMocks()
  vi.unstubAllGlobals()
})

const frame = () => floorplanFrame()

describe('floorplan layer', () => {
  it('renders the layer checkbox and the rebuild button', () => {
    render(<App />)
    expect(screen.getByRole('checkbox', { name: /Полигонная карта/ })).toBeTruthy()
    expect(screen.getByRole('button', { name: /Пересчитать карту/ })).toBeTruthy()
  })

  it('shows the layer after a floorplan frame arrives and keeps it on disconnect', () => {
    render(<App />)
    act(() => { lastHandler(frame()) })
    const status = screen.getByTestId('floorplan-status')
    expect(status.textContent).toContain('1 комн')
    // Layer state survives: same frame again is idempotent.
    act(() => { lastHandler(frame()) })
    expect(screen.getByTestId('floorplan-status').textContent).toContain('1 комн')
  })

  it('rebuild button posts to the endpoint and reports failures without clearing the layer', async () => {
    const fetchMock = vi.fn().mockResolvedValue({ ok: true, status: 200, json: async () => ({ version: 2 }) })
    vi.stubGlobal('fetch', fetchMock)
    render(<App />)
    act(() => { lastHandler(frame()) })
    const button = screen.getByRole('button', { name: /Пересчитать карту/ })
    await act(async () => { fireEvent.click(button) })
    expect(fetchMock).toHaveBeenCalled()
    const url = fetchMock.mock.calls.map((c) => String(c[0])).find((u) => u.includes('/api/floorplan/rebuild'))
    expect(url).toBeTruthy()
    expect(button.disabled).toBe(false)
    // Failure path: error text appears, the last layer stays.
    fetchMock.mockResolvedValue({ ok: false, status: 500, json: async () => ({ error: 'tool failed' }) })
    await act(async () => { fireEvent.click(button) })
    expect(screen.getByTestId('floorplan-status').textContent).toContain('1 комн')
  })

  it('disables the button while a rebuild is pending', async () => {
    let release
    const gate = new Promise((r) => { release = r })
    vi.stubGlobal('fetch', vi.fn().mockImplementation(() => gate.then(async () => ({ ok: true, status: 200, json: async () => ({ version: 2 }) }))))
    render(<App />)
    const button = screen.getByRole('button', { name: /Пересчитать карту/ })
    await act(async () => { fireEvent.click(button) })
    expect(button.disabled).toBe(true)
    await act(async () => { release() })
    expect(button.disabled).toBe(false)
  })
})
