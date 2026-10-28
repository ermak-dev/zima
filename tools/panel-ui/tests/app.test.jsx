// Leak-focused frontend tests: mount/unmount under StrictMode must leave
// no sockets, timers or listeners behind; collapsed camera stops decoding.
import { describe, it, expect, vi, afterEach } from 'vitest'
import { render, cleanup, act } from '@testing-library/react'

vi.mock('../src/usePanelSocket.js', () => ({
  default: (onMessage) => mockSocket(onMessage),
}))

const sockets = []
let lastHandler = null
let connected = true
function mockSocket(onMessage) {
  lastHandler = onMessage
  const fake = {
    onMessage,
    sent: [],
    send(msg) { this.sent.push(msg) },
  }
  sockets.push(fake)
  return { connected, send: (m) => fake.send(m) }
}

import App from '../src/App.jsx'

afterEach(() => {
  cleanup()
  sockets.length = 0
  connected = true
  vi.useRealTimers()
  vi.restoreAllMocks()
  vi.unstubAllGlobals()
})

describe('App lifecycle', () => {
  it('mounts sensor table with explicit missing values', () => {
    render(<App />)
    for (const id of ['battery', 'dock', 'bumper', 'wall', 'cliff-0', 'cliff-1', 'cliff-2', 'cliff-3', 'drop-0', 'drop-1', 'sonar', 'bumper-left', 'bumper-front', 'bumper-right', 'compass']) {
      expect(document.getElementById(id).textContent).toBe('нет данных')
    }
  })

  // BUG: native telemetry preserves unknown power fields as null.
  // HYPOTHESIS: checking only the battery object renders null as a measurement.
  // SOLUTION: format power fields independently and mark a wholly unknown reading missing.
  // RESULT: FIXED — unknown fields remain explicit; valid voltage is preserved.
  // STATUS: FIXED
  it('keeps unknown battery fields explicit instead of rendering null measurements', () => {
    render(<App />)
    act(() => lastHandler({ type: 'state', data: { sensors: { battery: { percent: null, volts: null, charging: null } } } }))
    expect(document.getElementById('battery').textContent).toBe('нет данных')
    expect(document.getElementById('battery').className).toBe('missing')
    act(() => lastHandler({ type: 'state', data: { sensors: { battery: { percent: null, volts: 20.5, charging: null } } } }))
    expect(document.getElementById('battery').textContent).toBe('—% / 20.5 V')
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

  it('renders per-segment bumper state', () => {
    render(<App />)
    act(() => lastHandler({ type: 'state', data: { sensors: { bumper: 'PRESSED', bumper_segments: [true, false, true] } } }))
    expect(document.getElementById('bumper').textContent).toBe('Нажат')
    expect(document.getElementById('bumper').className).toBe('alert')
    expect(document.getElementById('bumper-left').textContent).toBe('Нажат')
    expect(document.getElementById('bumper-left').className).toBe('alert')
    expect(document.getElementById('bumper-front').textContent).toBe('Свободен')
    expect(document.getElementById('bumper-right').textContent).toBe('Нажат')
    act(() => lastHandler({ type: 'state', data: { sensors: { bumper: 'clear', bumper_segments: [false, false, false] } } }))
    expect(document.getElementById('bumper-left').textContent).toBe('Свободен')
    expect(document.getElementById('bumper-left').className).toBe('')
  })

  it('renders compass yaw and dock state from telemetry', () => {
    render(<App />)
    act(() => lastHandler({ type: 'state', data: { sensors: { dock: true, compass: { x: 0, y: 0, z: 0, yaw: 1.256, stall: false } } } }))
    expect(document.getElementById('compass').textContent).toBe('1.256 рад · 72.0°')
    expect(document.getElementById('dock').textContent).toBe('На доке')
    expect(document.getElementById('dock').className).toBe('')
    act(() => lastHandler({ type: 'state', data: { sensors: { dock: false, compass: { yaw: null } } } }))
    expect(document.getElementById('compass').textContent).toBe('нет данных')
    expect(document.getElementById('compass').className).toBe('missing')
    expect(document.getElementById('dock').textContent).toBe('Вне дока')
  })

  // FEATURE: service telemetry with explicit missing and unsafe-subscription states.
  // SPEC: show every raw channel without converting flags into physical units.
  // IMPLEMENTATION: a collapsible read-only inventory distinguishes blocked subscriptions.
  // RESULT: IMPLEMENTED — missing values, raw arrays and inert viewing verified.
  // STATUS: DONE
  it('shows service channels without sending commands', () => {
    const { getByRole } = render(<App />)
    for (const key of ['ir:3', 'ir:4', 'ir:6', 'ir:7', 'ir:8', 'position3d:0', 'position3d:1', 'position3d:4']) {
      expect(document.getElementById(`service-${key}`).textContent).toBe('нет данных')
    }
    for (const key of ['motor:0', 'motor:1', 'motor:2']) {
      expect(document.getElementById(`service-${key}`).textContent).toMatch(/подписка отключена/)
    }
    expect(document.getElementById('service-keypad:0').textContent).toMatch(/формат не подтверждён/)
    act(() => lastHandler({ type: 'state', data: { sensors: { service_channels: {
      'ir:7': { voltages: [0, 12, null], ranges: [17], stamp: 42 },
      'position3d:0': { pos: [1, 2, 3, 0.1, 0.2, 0.3], vel: [0, 0, 0, 0, 0, null], stall: false },
    } } } }))
    expect(document.getElementById('service-ir:7').textContent).toBe('voltages [0, 12, —] · ranges [17]')
    expect(document.getElementById('service-position3d:0').textContent).toContain('pos [1, 2, 3, 0.1, 0.2, 0.3]')
    expect(getByRole('group', { name: 'Служебные каналы' })).toBeTruthy()
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

  it('speed slider scales teleop commands within the hard limits', () => {
    const { container } = render(<App />)
    const sent = () => sockets.flatMap((s) => s.sent)
    const slider = container.querySelector('#speed-slider')
    expect(slider).toBeTruthy()
    expect(slider.value).toBe('0.1')
    expect(slider.max).toBe('0.5')
    expect(document.getElementById('speed-value').textContent).toBe('0.10 м/с')
    // The default limit keeps the current behavior byte-for-byte.
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyW', bubbles: true })))
    expect(sent().at(-1)).toEqual({ v: 0.1, w: 0 })
    act(() => window.dispatchEvent(new KeyboardEvent('keyup', { code: 'KeyW', bubbles: true })))
    const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set
    act(() => {
      set.call(slider, '0.06')
      slider.dispatchEvent(new Event('input', { bubbles: true }))
    })
    expect(document.getElementById('speed-value').textContent).toBe('0.06 м/с')
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyW', bubbles: true })))
    expect(sent().at(-1)).toEqual({ v: 0.06, w: 0 })
    act(() => window.dispatchEvent(new KeyboardEvent('keyup', { code: 'KeyW', bubbles: true })))
    act(() => {
      set.call(slider, '0.5')
      slider.dispatchEvent(new Event('input', { bubbles: true }))
    })
    expect(document.getElementById('speed-value').textContent).toBe('0.50 м/с')
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyW', bubbles: true })))
    expect(sent().at(-1)).toEqual({ v: 0.5, w: 0 })
    expect(sent().every((c) => Math.abs(c.v) <= 0.5 && Math.abs(c.w) <= 0.4)).toBe(true)
    act(() => window.dispatchEvent(new KeyboardEvent('keyup', { code: 'KeyW', bubbles: true })))
  })

  // FEATURE: separate, labelled turn-speed slider without motion on adjustment.
  // SPEC: 0.01–0.4 rad/s, default 0.4; changing linear speed does not alter turn speed.
  // IMPLEMENTATION: separate turn-speed state and accessible slider in the footer.
  // RESULT: IMPLEMENTED — idle adjustment is inert and focused-slider WASD uses both limits.
  // STATUS: DONE
  it('adjusts turn speed independently without sending commands while idle', () => {
    const { getByRole } = render(<App />)
    const slider = getByRole('slider', { name: /Поворот/ })
    const sent = () => sockets.flatMap((s) => s.sent)
    expect(slider.min).toBe('0.01')
    expect(slider.max).toBe('0.4')
    expect(slider.step).toBe('0.01')
    expect(slider.value).toBe('0.4')
    expect(document.getElementById('turn-speed-value').textContent).toBe('0.40 рад/с')
    const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set
    act(() => {
      set.call(slider, '0.15')
      slider.dispatchEvent(new Event('input', { bubbles: true }))
    })
    expect(document.getElementById('turn-speed-value').textContent).toBe('0.15 рад/с')
    expect(sent()).toEqual([])
    const linear = document.getElementById('speed-slider')
    act(() => {
      set.call(linear, '0.5')
      linear.dispatchEvent(new Event('input', { bubbles: true }))
    })
    expect(sent()).toEqual([])
    act(() => slider.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyW', bubbles: true })))
    act(() => slider.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyA', bubbles: true })))
    expect(sent().at(-1)).toEqual({ v: 0.5, w: 0.15 })
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'Space', bubbles: true })))
    expect(sent().at(-1)).toEqual({ v: 0, w: 0 })
  })

  it('state messages render odometry telemetry', () => {
    render(<App />)
    act(() => lastHandler({ type: 'state', data: { odom: { x: 1.25, y: -0.5, vx: 0.1, yaw: 0, yaw_rate: 0.4 } } }))
    expect(document.getElementById('pose-x').textContent).toBe('1.25 м')
    expect(document.getElementById('pose-w').textContent).toBe('0.40 рад/с')
  })

  // FEATURE: Robot-local lidar replaces the occupancy map.
  // SPEC: No ROS/map UI; odometry is telemetry only; viewing sends no commands.
  // IMPLEMENTATION: Render incoming base_link points directly with a fixed robot origin.
  // RESULT: IMPLEMENTED — lidar-only controls preserve camera and keyboard safety.
  // STATUS: DONE
  it('shows only lidar, camera, sensors and telemetry without sending commands', () => {
    const { container, getByRole } = render(<App />)
    expect(getByRole('heading', { name: 'Лидар' })).toBeTruthy()
    expect(container.textContent).not.toMatch(/ROS|SLAM|TF|Ожидание карты|Свободно|Стены|Неизвестно/)
    expect(document.getElementById('frame-label').textContent).toBe('base_link')
    expect(container.querySelector('#map-fit, #map-scan, #map-message')).toBeNull()
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

  it('returns from expanded lidar on Escape and preserves collapse controls', () => {
    const { container } = render(<App />)
    act(() => container.querySelector('#lidar-expand').click())
    expect(container.querySelector('#workspace').dataset.expanded).toBe('lidar')
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'Escape' })))
    expect(container.querySelector('#workspace').dataset.expanded).toBe('')
    expect(document.activeElement.id).toBe('lidar-expand')
    act(() => container.querySelector('#lidar-collapse').click())
    expect(container.querySelector('#scan')).toBeNull()
    act(() => container.querySelector('#lidar-collapse').click())
    expect(container.querySelector('#scan')).toBeTruthy()
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

  it('draws incoming base_link points unchanged and keeps the robot at origin', () => {
    const frames = new Map()
    let next = 0
    vi.stubGlobal('requestAnimationFrame', (cb) => { frames.set(++next, cb); return next })
    vi.stubGlobal('cancelAnimationFrame', (id) => frames.delete(id))
    const flush = () => {
      const callbacks = [...frames.values()]
      frames.clear()
      callbacks.forEach((cb) => cb())
    }
    const ctx = Object.fromEntries(['setTransform', 'fillRect', 'beginPath', 'moveTo', 'lineTo', 'stroke', 'fillText', 'save', 'translate', 'rotate', 'arc', 'fill', 'restore', 'closePath'].map((name) => [name, vi.fn()]))
    vi.spyOn(HTMLCanvasElement.prototype, 'getContext').mockReturnValue(ctx)
    vi.spyOn(HTMLCanvasElement.prototype, 'getBoundingClientRect').mockReturnValue({ width: 654, height: 654 })
    const { unmount } = render(<App />)
    flush()
    expect(ctx.translate).toHaveBeenCalledWith(327, 327)
    ctx.arc.mockClear()
    ctx.translate.mockClear()
    act(() => {
      lastHandler({ type: 'state', data: { odom: { x: 100, y: 200, yaw: 1 } } })
      lastHandler({ type: 'scan', data: { points: [[1, 2], [-1, -2]], range_max: 6, frame: 'base_link', stamp: 123 } })
    })
    flush()
    expect(ctx.arc).toHaveBeenCalledWith(427, 127, 2, 0, 2 * Math.PI)
    expect(ctx.arc).toHaveBeenCalledWith(227, 527, 2, 0, 2 * Math.PI)
    expect(ctx.translate).toHaveBeenCalledWith(327, 327)
    ctx.arc.mockClear()
    act(() => lastHandler({ type: 'scan', data: { points: [], range_max: 6, frame: 'base_link', stamp: 124 } }))
    flush()
    expect(ctx.arc).not.toHaveBeenCalledWith(427, 127, 2, 0, 2 * Math.PI)
    const canvas = document.getElementById('scan')
    act(() => canvas.dispatchEvent(new WheelEvent('wheel', { deltaY: 100000 })))
    expect(document.getElementById('scale-value').textContent).toBe('30.0 м')
    act(() => canvas.dispatchEvent(new WheelEvent('wheel', { deltaY: -100000 })))
    expect(document.getElementById('scale-value').textContent).toBe('0.35 м')
    act(() => document.getElementById('lidar-center').click())
    flush()
    ctx.translate.mockClear()
    act(() => {
      canvas.dispatchEvent(new PointerEvent('pointerdown', { button: 0, pointerId: 1, clientX: 0, clientY: 0 }))
      canvas.dispatchEvent(new PointerEvent('pointermove', { pointerId: 1, clientX: 100, clientY: 0 }))
      canvas.dispatchEvent(new PointerEvent('pointerup', { pointerId: 1 }))
    })
    flush()
    expect(ctx.translate).toHaveBeenCalledWith(427, 327)
    act(() => canvas.dispatchEvent(new WheelEvent('wheel', { deltaY: 1 })))
    expect(frames.size).toBe(1)
    unmount()
    expect(frames.size).toBe(0)
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

  it('shows disconnect and reconnect without sending commands or restarting the camera', () => {
    vi.useFakeTimers()
    const { container, rerender } = render(<App />)
    const src = container.querySelector('#cam').getAttribute('src')
    connected = false
    act(() => vi.advanceTimersByTime(1000))
    rerender(<App />)
    expect(container.querySelector('#connection').textContent).toBe('Нет связи с панелью')
    connected = true
    rerender(<App />)
    expect(container.querySelector('#connection').textContent).toBe('Связь с роботом')
    expect(container.querySelector('#cam').getAttribute('src')).toBe(src)
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

  it('camera stream pauses when its panel is collapsed', () => {
    const { container } = render(<App />)
    const img = container.querySelector('#cam')
    expect(img).toBeTruthy()
    act(() => container.querySelector('#camera-collapse').click())
    expect(container.querySelector('#cam')).toBeNull()
  })
})
