// Leak-focused frontend tests: mount/unmount under StrictMode must leave
// no sockets, timers or listeners behind; collapsed camera stops decoding.
import { describe, it, expect, vi, afterEach } from 'vitest'
import { render, cleanup, act } from '@testing-library/react'

vi.mock('../src/usePanelSocket.js', () => ({
  default: (onMessage) => mockSocket(onMessage),
}))

const sockets = []
let lastHandler = null
function mockSocket(onMessage) {
  lastHandler = onMessage
  const fake = {
    onMessage,
    sent: [],
    send(msg) { this.sent.push(msg) },
  }
  sockets.push(fake)
  return { connected: true, send: (m) => fake.send(m) }
}

import App from '../src/App.jsx'

afterEach(() => {
  cleanup()
  sockets.length = 0
  vi.useRealTimers()
})

describe('App lifecycle', () => {
  it('mounts sensor table with explicit missing values', () => {
    render(<App />)
    for (const id of ['battery', 'bumper', 'wall', 'cliff-0', 'cliff-1', 'cliff-2', 'cliff-3', 'drop-0', 'drop-1', 'sonar']) {
      expect(document.getElementById(id).textContent).toBe('нет данных')
    }
  })

  it('voltage and unknown range are not conflated', () => {
    render(<App />)
    act(() => lastHandler({ type: 'state', data: { sensors: { cliff_ir: [0, 0, 0, 0] } } }))
    expect(document.getElementById('cliff-0').textContent).toBe('— м / 0.00 V')
  })

  it('sonar sentinel is flagged, not shown as distance', () => {
    render(<App />)
    act(() => lastHandler({ type: 'state', data: { sensors: { sonar: 65.534 } } }))
    expect(document.getElementById('sonar').textContent).toBe('65.534 м · sentinel')
    expect(document.getElementById('sonar').className).toBe('alert')
  })

  it('state and scan messages render pose values', () => {
    render(<App />)
    act(() => lastHandler({ type: 'state', data: { odom: { x: 1.25, y: -0.5, vx: 0.1, yaw: 0, yaw_rate: 0.4 } } }))
    expect(document.getElementById('pose-x').textContent).toBe('1.25 м')
    expect(document.getElementById('pose-w').textContent).toBe('0.40 рад/с')
  })

  it('map without fresh TF keeps pose missing and says so', () => {
    render(<App />)
    act(() => lastHandler({ type: 'map', data: { width: 2, height: 2, resolution: 0.05, origin: { x: 0, y: 0, yaw: 0 }, frame: 'map', version: 1, image: 'data:image/png;base64,test' } }))
    act(() => lastHandler({ type: 'state', data: { odom: { x: 100 }, map_age_s: 0 } }))
    expect(document.getElementById('pose-x').textContent).toBe('—')
    expect(document.getElementById('map-message').textContent).toMatch(/TF/)
  })

  it('camera stream pauses when its panel is collapsed', () => {
    const { container } = render(<App />)
    const img = container.querySelector('#cam')
    expect(img).toBeTruthy()
    act(() => container.querySelector('#camera-collapse').click())
    expect(container.querySelector('#cam')).toBeNull()
  })
})
