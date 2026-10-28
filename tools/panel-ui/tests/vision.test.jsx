// FEATURE: YOLO camera overlay on the exact JPEG used for inference.
// SPEC: Config on control WS; separate readonly video WS; atomic decoded frames,
// safe normalized boxes, stale/disconnect clearing and bounded lifecycle cleanup.
// IMPLEMENTATION: Independent readonly hook decodes JPEG before committing it with
// validated detections; a shared SVG viewBox preserves image/box letterboxing.
// RESULT: IMPLEMENTED — config, atomic pairing, escaping, stale clearing,
// reconnection and StrictMode/collapse cleanup verified with local fakes.
// STATUS: DONE
import { StrictMode } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, render } from '@testing-library/react'
import App from '../src/App.jsx'
import VisionFrame from '../src/VisionFrame.jsx'

class FakeSocket {
  static OPEN = 1
  static instances = []
  constructor(url) { this.url = url; this.readyState = 1; this.sent = []; FakeSocket.instances.push(this) }
  send(data) { this.sent.push(JSON.parse(data)) }
  close() { this.closed = true; this.readyState = 3; this.onclose?.() }
  drop() { this.close() }
  message(data) { act(() => this.onmessage?.({ data: JSON.stringify(data) })) }
}
class FakeImage {
  static instances = []
  constructor() { FakeImage.instances.push(this) }
  set src(value) { this.source = value }
  get src() { return this.source }
}
const control = () => FakeSocket.instances.find((ws) => ws.url.endsWith('/ws') && !ws.url.endsWith('/vision/ws'))
const video = () => FakeSocket.instances.filter((ws) => ws.url.endsWith('/vision/ws')).at(-1)
const config = (enabled) => control().message({ type: 'vision', data: { enabled } })
const frame = (id, extra = {}) => ({ type: 'vision', data: {
  status: 'live', frame_id: id, jpeg: btoa(`jpeg-${id}`), width: 640, height: 360,
  objects: [{ label: `объект-${id}`, confidence: 0.91, box: [0.1, 0.2, 0.7, 0.8] }], inference_ms: 42, ...extra,
} })
const decode = () => act(() => FakeImage.instances.at(-1).onload?.())
const image = () => document.querySelector('.vision-frame image')

beforeEach(() => {
  vi.useFakeTimers()
  FakeSocket.instances = []; FakeImage.instances = []
  vi.stubGlobal('WebSocket', FakeSocket); vi.stubGlobal('Image', FakeImage)
})
afterEach(() => { cleanup(); vi.useRealTimers(); vi.restoreAllMocks(); vi.unstubAllGlobals() })

describe('YOLO camera', () => {
  // BUG: portrait images shrink label text with the SVG viewBox into unreadability.
  // SPEC: labels stay 14 CSS pixels through letterboxing and panel resizing.
  it('keeps labels readable on letterboxed portrait frames and releases resize observer', () => {
    let size = { width: 400, height: 300 }
    let resized
    const disconnected = vi.fn()
    vi.spyOn(SVGElement.prototype, 'getBoundingClientRect').mockImplementation(() => size)
    vi.stubGlobal('ResizeObserver', class {
      constructor(callback) { resized = callback }
      observe() {}
      disconnect() { disconnected() }
    })
    const { unmount } = render(<VisionFrame frame={{ ...frame(1, { width: 810, height: 1080 }).data, src: 'data:image/jpeg;base64,eA==' }} />)
    const text = () => document.querySelector('.vision-frame text')
    expect(Number(text().getAttribute('font-size')) * Math.min(400 / 810, 300 / 1080)).toBeCloseTo(14)
    size = { width: 900, height: 800 }
    act(() => resized())
    expect(Number(text().getAttribute('font-size')) * Math.min(900 / 810, 800 / 1080)).toBeCloseTo(14)
    unmount()
    expect(disconnected).toHaveBeenCalledTimes(1)
  })
  it('uses config from control WS and never requests raw MJPEG while enabled', () => {
    const { unmount } = render(<App />)
    const raw = document.querySelector('#cam').getAttribute('src')
    expect(raw).toMatch(/^\/camera.mjpeg/)
    act(() => control().onopen?.())
    config(true)
    expect(video()?.url).toBe(`ws://${location.host}/vision/ws`)
    expect(document.querySelector('#cam')).toBeNull()
    expect(FakeImage.instances.filter((img) => img.src?.includes('camera.mjpeg'))).toHaveLength(0)
    expect(document.querySelector('#cam-status').textContent).toContain('YOLO')
    expect(control().sent).toEqual([])
    expect(video().sent).toEqual([])
    config(false)
    expect(video().closed).toBe(true)
    expect(document.querySelector('#cam').getAttribute('src')).toMatch(/^\/camera.mjpeg/)
    unmount()
  })

  it('commits JPEG and boxes atomically after decode and ignores out-of-order callbacks', () => {
    render(<App />); config(true)
    video().message(frame(1))
    const first = FakeImage.instances.at(-1)
    const oldLoad = first.onload
    expect(image()).toBeNull()
    video().message(frame(2))
    decode()
    expect(image().getAttribute('href')).toBe(`data:image/jpeg;base64,${btoa('jpeg-2')}`)
    expect(document.querySelector('.vision-frame').textContent).toContain('объект-2 91%')
    act(() => oldLoad?.())
    expect(document.querySelector('.vision-frame').textContent).not.toContain('объект-1')
    expect(document.querySelector('.vision-frame').getAttribute('viewBox')).toBe('0 0 640 360')
    expect(document.querySelector('.vision-frame').getAttribute('preserveAspectRatio')).toBe('xMidYMid meet')
    expect(document.querySelector('.vision-frame rect').getAttribute('x')).toBe('64')
    expect(document.querySelector('#cam-status').textContent).toContain('42 мс')
    video().message(frame(3)); expect(image().getAttribute('href')).toContain(btoa('jpeg-2'))
    decode(); expect(image().getAttribute('href')).toContain(btoa('jpeg-3'))
    video().message(frame(2)); decode()
    expect(image().getAttribute('href')).toContain(btoa('jpeg-3'))
  })

  it('escapes labels and ignores invalid boxes without discarding valid detections', () => {
    render(<App />); config(true)
    const label = '<img src=x onerror=alert(1)>'
    video().message(frame(1, { objects: [
      { label, confidence: 0.5, box: [0, 0, 1, 1] },
      ...[[0.8, 0, 0.2, 1], [-0.1, 0, 1, 1], [0, 0, 2, 1], [0, 0, 0, 1], [0, 0, null, 1], [0, 0, 1]].map((box) => ({ label: 'bad', confidence: 0.9, box })),
    ] }))
    decode()
    expect(document.querySelectorAll('.vision-frame rect')).toHaveLength(1)
    expect(document.querySelector('.vision-frame').textContent).toContain(label)
    expect(document.querySelector('.vision-frame img')).toBeNull()
    expect(document.querySelector('.vision-frame').textContent).not.toContain('bad')
  })

  it('clears stale frames and pending decodes, handles loading/error and reconnects video alone', () => {
    render(<App />); config(true)
    const ws = video(); video().message(frame(1)); decode()
    expect(image()).toBeTruthy()
    act(() => vi.advanceTimersByTime(3100))
    expect(image()).toBeNull()
    expect(document.querySelector('#cam-status').textContent).toContain('Нет свежих кадров')
    ws.message(frame(2)); const late = FakeImage.instances.at(-1).onload
    act(() => vi.advanceTimersByTime(3100)); act(() => late?.())
    expect(image()).toBeNull()
    ws.message(frame(3)); decode()
    ws.message({ type: 'vision', data: { status: 'loading' } })
    expect(image()).toBeNull()
    ws.message({ type: 'vision', data: { status: 'error', error: 'Модель недоступна' } })
    expect(document.querySelector('#cam-off').textContent).toContain('Модель недоступна')
    ws.message(frame(4)); decode()
    act(() => ws.drop())
    expect(image()).toBeNull()
    act(() => vi.advanceTimersByTime(1000))
    expect(video()).not.toBe(ws)
    expect(FakeSocket.instances.filter((s) => !s.url.includes('/vision/'))).toHaveLength(1)
    expect(control().sent).toEqual([])
  })

  it('bounds backoff and stops sockets, images and timers on collapse, hidden panels and StrictMode unmount', () => {
    const { container, unmount } = render(<StrictMode><App /></StrictMode>)
    const currentControl = FakeSocket.instances.filter((s) => !s.url.includes('/vision/')).at(-1)
    currentControl.message({ type: 'vision', data: { enabled: true } })
    for (const delay of [500, 1000, 2000, 4000, 5000, 5000]) {
      const ws = video(); act(() => ws.drop())
      act(() => vi.advanceTimersByTime(delay))
      expect(video()).not.toBe(ws)
    }
    video().message(frame(1)); const pending = FakeImage.instances.at(-1)
    const late = pending.onload
    act(() => container.querySelector('#camera-collapse').click())
    expect(video().closed).toBe(true)
    expect(pending.src).toBe('')
    act(() => late?.()); expect(image()).toBeNull()
    act(() => container.querySelector('#camera-collapse').click())
    expect(video().closed).not.toBe(true)
    act(() => container.querySelector('#lidar-expand').click())
    expect(video().closed).toBe(true)
    act(() => container.querySelector('#lidar-expand').click())
    unmount()
    const count = FakeSocket.instances.length
    act(() => vi.advanceTimersByTime(30000))
    expect(FakeSocket.instances).toHaveLength(count)
    expect(FakeSocket.instances.every((s) => s.closed)).toBe(true)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('releases all video listeners on disconnect and cannot restart after collapse during backoff', () => {
    const { container } = render(<App />); config(true)
    const ws = video()
    act(() => ws.drop())
    expect(ws.onmessage).toBeNull()
    expect(ws.onclose).toBeNull()
    expect(ws.onerror).toBeNull()
    act(() => container.querySelector('#camera-collapse').click())
    const count = FakeSocket.instances.length
    act(() => vi.advanceTimersByTime(30000))
    expect(FakeSocket.instances).toHaveLength(count)
  })

  it('ignores malformed packets and clears failed decodes rather than showing mismatched marks', () => {
    render(<App />); config(true)
    const ws = video()
    const before = FakeImage.instances.length
    act(() => ws.onmessage({ data: 'not JSON' }))
    for (const data of [null, {}, { type: 'state', data: {} }, frame(0, { width: 0 }), frame(0, { jpeg: '' }), frame(0, { objects: null })]) ws.message(data)
    expect(FakeImage.instances).toHaveLength(before)
    ws.message(frame(1, { objects: [] })); decode()
    expect(image()).toBeTruthy()
    expect(document.querySelectorAll('.vision-frame rect')).toHaveLength(0)
    ws.message(frame(2)); const pending = FakeImage.instances.at(-1)
    act(() => pending.onerror())
    expect(image()).toBeNull()
    expect(document.querySelector('#cam-off').textContent).toContain('Не удалось декодировать кадр')
    ws.message(frame(3)); const late = FakeImage.instances.at(-1).onload
    ws.message({ type: 'vision', data: { status: 'error', error: '<script>bad()</script>' } })
    act(() => late())
    expect(image()).toBeNull()
    expect(document.querySelector('#cam-off').textContent).toContain('<script>bad()</script>')
    expect(document.querySelector('#cam-off script')).toBeNull()
  })

  it('applies fresh enabled config after a control reconnect without changing drive messages', () => {
    render(<App />); config(true)
    const old = control()
    act(() => old.drop())
    act(() => vi.advanceTimersByTime(1000))
    const next = FakeSocket.instances.filter((s) => !s.url.includes('/vision/')).at(-1)
    next.message({ type: 'vision', data: { enabled: false } })
    expect(video().closed).toBe(true)
    expect(document.querySelector('#cam')).toBeTruthy()
    act(() => next.onopen?.())
    act(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'KeyW', bubbles: true })))
    expect(next.sent.at(-1)).toEqual({ v: 0.1, w: 0 })
    act(() => window.dispatchEvent(new KeyboardEvent('keyup', { code: 'KeyW', bubbles: true })))
  })
})
