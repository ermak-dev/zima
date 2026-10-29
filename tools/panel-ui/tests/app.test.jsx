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

const rawCompass = (yaw) => ({
  source: 'st480-evdev', status: 'uncalibrated', calibrated: false, stamp: 42,
  x: Math.round(100 * Math.cos(yaw)), y: Math.round(100 * Math.sin(yaw)), z: 20, yaw,
})

afterEach(() => {
  cleanup()
  sockets.length = 0
  connected = true
  vi.useRealTimers()
  vi.restoreAllMocks()
  vi.unstubAllGlobals()
})

describe('App lifecycle', () => {
  // FEATURE: Zima header with the supplied logo embedded in the single-file panel.
  // SPEC: replace Roborock text and the generic SVG without an extra image request.
  // IMPLEMENTATION: inline PNG asset with responsive header dimensions.
  // RESULT: IMPLEMENTED — Zima header and embedded PNG verified; original bytes preserved.
  // STATUS: DONE
  it('brands the header as Zima with its embedded PNG logo', () => {
    const { container, getByRole } = render(<App />)
    const brand = container.querySelector('header .brand')
    expect(brand.textContent).toContain('Zima')
    expect(brand.textContent).not.toContain('Roborock')
    const logo = getByRole('img', { name: 'Логотип Zima' })
    expect(brand.contains(logo)).toBe(true)
    expect(logo.getAttribute('src')).toMatch(/^data:image\/png;base64,/)
    expect(brand.querySelector('svg')).toBeNull()
  })

  // FEATURE: obstacle balloon with an explicit server-confirmed reset.
  // SPEC: state.blocked is authoritative, held keys are cancelled, and reset never resumes motion.
  // IMPLEMENTATION: persistent alert sends typed reset and waits for server state.
  // RESULT: IMPLEMENTED — no auto-resume, camera unchanged and disconnected reset disabled.
  // STATUS: DONE
  it('shows a reset balloon and waits for acknowledgement without restarting movement', () => {
    vi.useFakeTimers()
    const { getByRole, queryByRole } = render(<App />)
    const sent = () => sockets.flatMap((s) => s.sent)
    const cameraSrc = document.getElementById('cam').getAttribute('src')
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyW', bubbles: true })))
    expect(sent().at(-1)).toEqual({ v: 0.1, w: 0 })
    act(() => lastHandler({ type: 'state', data: { blocked: true, sensors: { bumper: 'PRESSED' } } }))
    expect(document.getElementById('cam').getAttribute('src')).toBe(cameraSrc)
    expect(getByRole('alert').textContent).toContain('Наезд на препятствие')
    const count = sent().length
    act(() => vi.advanceTimersByTime(450))
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyS', bubbles: true })))
    expect(sent()).toHaveLength(count)
    act(() => getByRole('button', { name: 'Сброс' }).click())
    expect(sent().at(-1)).toEqual({ type: 'reset_blocked' })
    expect(getByRole('alert')).toBeTruthy()
    const afterReset = sent().length
    act(() => lastHandler({ type: 'state', data: { blocked: false, sensors: { bumper: 'PRESSED' } } }))
    expect(queryByRole('alert')).toBeNull()
    act(() => vi.advanceTimersByTime(450))
    expect(sent()).toHaveLength(afterReset)
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyS', repeat: true, bubbles: true })))
    expect(sent()).toHaveLength(afterReset)
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyS', bubbles: true })))
    expect(sent().at(-1)).toEqual({ v: -0.1, w: 0 })
    act(() => window.dispatchEvent(new KeyboardEvent('keyup', { code: 'KeyS', bubbles: true })))
  })

  it('disables reset while disconnected and preserves blocked notification', () => {
    connected = false
    const { getByRole } = render(<App />)
    act(() => lastHandler({ type: 'state', data: { blocked: true } }))
    expect(getByRole('button', { name: 'Сброс' }).disabled).toBe(true)
    act(() => getByRole('button', { name: 'Сброс' }).click())
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

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

  // BUG: vendor yaw was treated as a measured course and could remain visible indefinitely.
  // HYPOTHESIS: only fresh ST480 samples may drive the needle; raw-field angles are not north.
  // SOLUTION: validate the source and expire unchanged sample stamps independently of other telemetry.
  // RESULT: FIXED — vendor yaw rejected, raw angle rendered and stale sample hidden.
  // STATUS: FIXED
  it('rejects nonzero vendor compass yaw rather than displaying a fabricated heading', () => {
    const { container } = render(<App />)
    act(() => lastHandler({ type: 'state', data: { sensors: {
      compass: { x: 600, y: 1, z: 2, yaw: 1.25, status: 'received' },
    } } }))
    expect(container.querySelector('[data-compass-needle]')).toBeNull()
    expect(document.getElementById('compass').textContent).toBe('нет данных')
  })

  it('expires a raw compass sample even while unrelated state frames keep arriving', () => {
    vi.useFakeTimers()
    const { container, getByRole } = render(<App />)
    const compass = { x: 0, y: 100, z: 20, yaw: Math.PI / 2, stamp: 42,
      source: 'st480-evdev', status: 'uncalibrated', calibrated: false }
    const publish = (sample = compass) => act(() => lastHandler({ type: 'state', data: { sensors: { compass: sample } } }))
    publish()
    expect(container.querySelector('[data-compass-needle]').getAttribute('transform')).toBe('rotate(-90 60 60)')
    expect(getByRole('img', { name: 'Компас' }).querySelector('title').textContent).toContain('осях датчика')
    expect(container.querySelector('.orientation-instruments').textContent).toContain('Без калибровки')
    act(() => vi.advanceTimersByTime(1000))
    publish({ ...compass })
    act(() => vi.advanceTimersByTime(600))
    expect(container.querySelector('[data-compass-needle]')).toBeNull()
    expect(document.getElementById('compass').textContent).toBe('нет данных')
    expect(container.querySelector('.orientation-instruments').textContent).toContain('Нет свежих измерений')
    publish({ ...compass, stamp: 43, yaw: 0, x: 100, y: 0 })
    expect(container.querySelector('[data-compass-needle]').getAttribute('transform')).toBe('rotate(0 60 60)')
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

  it('renders compass yaw and dock state from telemetry', () => {
    render(<App />)
    act(() => lastHandler({ type: 'state', data: { sensors: { dock: true, compass: rawCompass(1.256) } } }))
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
  // IMPLEMENTATION: an always-visible read-only inventory distinguishes blocked subscriptions.
  // RESULT: IMPLEMENTED — missing values, raw arrays and inert viewing verified.
  // STATUS: DONE
  it('shows service channels without sending commands', () => {
    const { getByRole } = render(<App />)
    for (const key of ['ir:3', 'ir:4', 'ir:6', 'ir:7', 'ir:8']) {
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
    expect(document.getElementById('service-position3d:0')).toBeNull()
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
  // SPEC: 0.01–3.14 rad/s, default 0.4; changing linear speed does not alter turn speed.
  // IMPLEMENTATION: separate turn-speed state and accessible slider in the footer.
  // RESULT: IMPLEMENTED — idle adjustment is inert and focused-slider WASD uses both limits.
  // STATUS: DONE
  it('adjusts turn speed independently without sending commands while idle', () => {
    const { getByRole } = render(<App />)
    const slider = getByRole('slider', { name: /Поворот/ })
    const sent = () => sockets.flatMap((s) => s.sent)
    expect(slider.min).toBe('0.01')
    expect(slider.max).toBe('3.14')
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

  // FEATURE: always-visible orientation instruments and dense telemetry.
  // SPEC: missing stays unknown; sensor angles drive the graphics; service channels need no expansion.
  // IMPLEMENTATION: two labelled SVG instruments, root viewport grid and open service inventory.
  // RESULT: IMPLEMENTED — angles, unknown states and inert rendering verified offline.
  // STATUS: DONE
  it('shows compass and gyro instruments with real angles and explicit unknowns', () => {
    const { getByRole, container } = render(<App />)
    expect(getByRole('img', { name: 'Компас' }).dataset.known).toBe('false')
    expect(getByRole('img', { name: 'Гироскоп' }).dataset.known).toBe('false')
    expect(container.querySelector('[data-compass-needle]')).toBeNull()
    act(() => lastHandler({ type: 'state', data: { sensors: {
      compass: rawCompass(Math.PI / 2),
      service_channels: { 'position3d:0': { pos: [0, 0, 0, Math.PI / 6, Math.PI / 12, -Math.PI / 4], vel: [0, 0, 0, 0, 0, 0] } },
    } } }))
    expect(getByRole('img', { name: 'Компас' }).dataset.known).toBe('true')
    expect(container.querySelector('[data-compass-needle]').getAttribute('transform')).toBe('rotate(-90 60 60)')
    const gyro = getByRole('img', { name: 'Гироскоп' })
    expect(gyro.dataset.yaw).toBe('-45')
    expect(container.querySelector('[data-gyro-heading]').getAttribute('transform')).toBe('rotate(45 60 60)')
    expect(document.getElementById('gyro-yaw').textContent).toBe('-45.0°')
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

  // FEATURE: planar gyro yaw and an honest compass-unavailable state.
  // SPEC: gyro depends on yaw, not roll/pitch; zero placeholders are not measured north.
  // IMPLEMENTATION: top-down yaw glyph, explicit empty compass and no duplicate gyro row.
  // RESULT: IMPLEMENTED — planar rotation, unknown compass and valid zero heading verified.
  // STATUS: DONE
  it('uses planar yaw and rejects empty compass frames without rejecting real zero heading', () => {
    const { getByRole, container } = render(<App />)
    act(() => lastHandler({ type: 'state', data: { sensors: {
      compass: { x: 0, y: 0, z: 0, yaw: 0 },
      service_channels: { 'position3d:0': { pos: [0, 0, 0, null, null, Math.PI / 2], vel: [0, 0, 0, 0, 0, 0.25] } },
    } } }))
    expect(getByRole('img', { name: 'Компас' }).dataset.known).toBe('false')
    expect(container.querySelector('[data-compass-needle]')).toBeNull()
    expect(document.getElementById('compass').textContent).toBe('нет данных')
    expect(getByRole('img', { name: 'Гироскоп' }).dataset.known).toBe('true')
    expect(container.querySelector('[data-gyro-heading]').getAttribute('transform')).toBe('rotate(-90 60 60)')
    expect(container.querySelector('.attitude-horizon')).toBeNull()
    expect(getByRole('group', { name: 'Служебные каналы' }).textContent).not.toContain('Гироскоп')
    act(() => lastHandler({ type: 'state', data: { sensors: { compass: rawCompass(0) } } }))
    expect(getByRole('img', { name: 'Компас' }).dataset.known).toBe('true')
    expect(container.querySelector('[data-compass-needle]').getAttribute('transform')).toBe('rotate(0 60 60)')
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
  })

  it('keeps every service value exposed without a collapsed disclosure', () => {
    const { getByRole, container } = render(<App />)
    const service = getByRole('group', { name: 'Служебные каналы' })
    expect(service.closest('details:not([open])')).toBeNull()
    expect(container.querySelector('.sensor-readings')).toBeTruthy()
    expect(container.querySelector('.orientation-instruments')).toBeTruthy()
    expect(service.querySelectorAll('[id^="service-"]').length).toBe(9)
  })

  it('allows the chassis turn ceiling without changing the initial speed', () => {
    const { getByRole } = render(<App />)
    const slider = getByRole('slider', { name: /Поворот/ })
    expect(slider.max).toBe('3.14')
    expect(slider.value).toBe('0.4')
    const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set
    act(() => {
      set.call(slider, '3.14')
      slider.dispatchEvent(new Event('input', { bubbles: true }))
    })
    expect(document.getElementById('turn-speed-value').textContent).toBe('3.14 рад/с')
    act(() => slider.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyA', bubbles: true })))
    expect(sockets.flatMap((s) => s.sent).at(-1)).toEqual({ v: 0, w: 3.14 })
    act(() => slider.dispatchEvent(new KeyboardEvent('keyup', { code: 'KeyA', bubbles: true })))
  })

  // FEATURE: dedicated position and measured-velocity odometry instruments.
  // SPEC: preserve unknowns and signed values; remove duplicate service rows and lidar strip.
  // IMPLEMENTATION: position/course graphic and two measured-velocity meters next to orientation.
  // RESULT: IMPLEMENTED — missing data, signed meters, numeric values and deduplication verified.
  // STATUS: DONE
  it('shows odometry as instruments instead of system information', () => {
    const { getByRole, container } = render(<App />)
    const pose = getByRole('img', { name: 'Положение одометрии' })
    const velocity = getByRole('img', { name: 'Скорости одометрии' })
    expect(pose.dataset.known).toBe('false')
    expect(velocity.dataset.known).toBe('false')
    expect(container.querySelector('[data-odometry-position]')).toBeNull()
    expect(document.getElementById('pose-v').textContent).toBe('—')
    const services = getByRole('group', { name: 'Служебные каналы' })
    expect(services.textContent).not.toMatch(/Одометрия|position3d:1|position3d:4/)
    expect(container.querySelector('#panel-lidar #odom')).toBeNull()
    act(() => lastHandler({ type: 'state', data: { odom: { x: 3, y: 4, yaw: Math.PI / 2, vx: -0.25, yaw_rate: 1.57 } } }))
    expect(pose.dataset.known).toBe('true')
    expect(velocity.dataset.known).toBe('true')
    expect(container.querySelector('[data-odometry-position]').dataset.x).toBe('3')
    expect(container.querySelector('[data-odometry-position]').dataset.y).toBe('4')
    expect(document.getElementById('pose-x').textContent).toBe('3.00 м')
    expect(document.getElementById('pose-v').textContent).toBe('-0.25 м/с')
    expect(document.getElementById('pose-w').textContent).toBe('1.57 рад/с')
    expect(container.querySelector('[data-odometry-linear]').dataset.ratio).toBe('-0.5')
    expect(container.querySelector('[data-odometry-angular]').dataset.ratio).toBe('0.5')
    expect(sockets.flatMap((s) => s.sent)).toEqual([])
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

  // FEATURE: Fixed-map diagnostic view is local-only and preserves manual controls.
  // SPEC: frame toggle resets pan; centering keeps the selected frame; no robot commands.
  // IMPLEMENTATION: local view-frame toggle, preserving manual control and instrument components.
  // RESULT: IMPLEMENTED — toggle test red before implementation; existing control suite stays green.
  // STATUS: DONE
  it('toggles map diagnostics locally without replacing branding or instruments', () => {
    const { getByRole } = render(<App />)
    const frame = getByRole('button', { name: 'Кадр карты' })
    act(() => frame.click())
    expect(frame.getAttribute('aria-pressed')).toBe('true')
    expect(document.getElementById('frame-label').textContent).toBe('map')
    act(() => document.getElementById('lidar-center').click())
    expect(document.getElementById('frame-label').textContent).toBe('map')
    expect(getByRole('img', { name: 'Логотип Zima' })).toBeTruthy()
    expect(getByRole('img', { name: 'Положение одометрии' })).toBeTruthy()
    act(() => frame.click())
    expect(document.getElementById('frame-label').textContent).toBe('base_link')
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
