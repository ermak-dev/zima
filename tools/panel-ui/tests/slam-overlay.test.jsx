// FEATURE: Scan/pose coherence and a fixed-map diagnostic frame.
// SPEC: Robot-frame map overlay hides on mismatch/stale/error/lost; fixed-map
// occupancy stays visible independently, but raw scan needs a matching valid pose.
// IMPLEMENTATION: coherence gates plus an App-owned age clock survive canvas remounts.
// RESULT: IMPLEMENTED — mismatch, timeout, error, reused stamp and remount regressions red/green.
// STATUS: DONE
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, render } from '@testing-library/react'
import LidarCanvas from '../src/LidarCanvas.jsx'
import App from '../src/App.jsx'

let onMessage
vi.mock('../src/usePanelSocket.js', () => ({ default: (handler) => {
  onMessage = handler
  return { connected: true, send: () => {} }
} }))

let ctx, imageCtx
const scan = { id: 8, frame: 'base_link', points: [[1, 0], [0, -2]] }
const slam = {
  scan_id: 8, scan_stamp: 9, age_ms: 0, max_age_ms: 500, stale: false,
  mode: 'mapping', tracking: 'tracking', pose: { x: 2, y: -1, yaw: Math.PI / 2 },
  map: { x_min: 0, y_min: 0, w: 2, h: 1, res: 1, cells: '100,-1' },
}
const view = { radius: 3, center: null }
function draw() { act(() => vi.advanceTimersByTime(20)) }
function mount(s = slam, frame = 'robot', sc = scan) {
  return render(<LidarCanvas scan={sc} slam={s} view={{ ...view, frame }} onView={() => {}} />)
}

beforeEach(() => {
  vi.useFakeTimers()
  ctx = Object.fromEntries(['setTransform', 'fillRect', 'beginPath', 'moveTo', 'lineTo', 'stroke', 'fillText',
    'save', 'restore', 'drawImage', 'arc', 'fill', 'translate', 'rotate', 'closePath'].map((k) => [k, vi.fn()]))
  imageCtx = { createImageData: (w, h) => ({ data: new Uint8ClampedArray(w * h * 4) }), putImageData: vi.fn() }
  vi.spyOn(HTMLCanvasElement.prototype, 'getContext').mockImplementation(function () { return this.id === 'scan' ? ctx : imageCtx })
  vi.spyOn(HTMLCanvasElement.prototype, 'getBoundingClientRect').mockReturnValue({ width: 254, height: 254 })
  vi.stubGlobal('requestAnimationFrame', (fn) => setTimeout(fn, 1))
  vi.stubGlobal('cancelAnimationFrame', clearTimeout)
})
afterEach(() => { cleanup(); vi.useRealTimers(); vi.restoreAllMocks(); vi.unstubAllGlobals() })

describe('SLAM overlay rendering', () => {
  it('renders aligned map and preserves unknown grid gaps', () => {
    mount(); draw()
    expect(ctx.drawImage).toHaveBeenCalledTimes(1)
    const pixels = imageCtx.putImageData.mock.calls[0][0].data
    expect(pixels[3]).toBeGreaterThan(0)
    expect(pixels[7]).toBe(0)
  })

  it.each([
    { scan_id: 7 }, { stale: true }, { tracking: 'error' }, { tracking: 'lost' },
    { mode: 'lost' }, { error: 'adapter timeout' }, { age_ms: 501 },
  ])('hides the robot-frame map for %j but preserves raw scan', (change) => {
    mount({ ...slam, ...change }); draw()
    expect(ctx.drawImage).not.toHaveBeenCalled()
    expect(ctx.arc).toHaveBeenCalledWith(127 + 100 / 3, 127, 2, 0, 2 * Math.PI)
  })

  it('hides overlay on a local timeout, even with repeated unchanged telemetry', () => {
    const { rerender } = mount(); draw()
    expect(ctx.drawImage).toHaveBeenCalledTimes(1)
    act(() => vi.advanceTimersByTime(300))
    rerender(<LidarCanvas scan={scan} slam={{ ...slam }} view={view} onView={() => {}} />)
    draw()
    ctx.drawImage.mockClear()
    act(() => vi.advanceTimersByTime(350))
    rerender(<LidarCanvas scan={scan} slam={{ ...slam }} view={{ ...view, radius: 3.1 }} onView={() => {}} />)
    draw()
    expect(ctx.drawImage).not.toHaveBeenCalled()
  })

  it('uses server sequence identity when source IDs and stamps are reused', () => {
    const first = { ...slam, scan_sequence: 101 }
    const firstScan = { ...scan, sequence: 101 }
    const { rerender } = mount(first, 'robot', firstScan); draw()
    act(() => vi.advanceTimersByTime(650))
    ctx.drawImage.mockClear()
    const next = { ...slam, scan_sequence: 102 }
    rerender(<LidarCanvas scan={{ ...scan, sequence: 102 }} slam={next} view={view} onView={() => {}} />)
    draw()
    expect(ctx.drawImage).toHaveBeenCalled()
    ctx.drawImage.mockClear()
    rerender(<LidarCanvas scan={{ ...scan, sequence: 103 }} slam={next} view={view} onView={() => {}} />)
    draw()
    expect(ctx.drawImage).not.toHaveBeenCalled()
  })

  it('uses world coordinates and robot yaw in fixed-map frame', () => {
    mount(slam, 'map'); draw()
    expect(ctx.drawImage).toHaveBeenCalled()
    const arc = ctx.arc.mock.calls.find((args) => args[2] === 2)
    expect(arc[0]).toBeCloseTo(127 + 2 * 100 / 3)
    expect(arc[1]).toBeCloseTo(127)
    expect(ctx.translate).toHaveBeenCalledWith(127 + 2 * 100 / 3, 127 + 100 / 3)
    // The marker must agree with its own scan: base_link +x (forward) is
    // screen right at yaw 0, so only -yaw rotates the marker, no extra -90°.
    expect(ctx.rotate).toHaveBeenCalledWith(-Math.PI / 2)
  })

  it('points the robot marker along the scan forward axis in robot frame', () => {
    mount(slam, 'robot'); draw()
    expect(ctx.translate).toHaveBeenCalledWith(127, 127)
    expect(ctx.rotate).toHaveBeenCalledWith(0)
  })

  it.each([{ scan_id: 7 }, { stale: true }, { error: 'timeout' }, { mode: 'lost' }])(
    'keeps fixed occupancy but hides unaligned world scan and robot for %j', (change) => {
      mount({ ...slam, ...change }, 'map'); draw()
      expect(ctx.drawImage).toHaveBeenCalled()
      expect(ctx.arc).not.toHaveBeenCalled()
    })

  it('does not align reused IDs when available source stamps differ', () => {
    mount(slam, 'robot', { ...scan, stamp: 8 }); draw()
    expect(ctx.drawImage).not.toHaveBeenCalled()
  })

  it('does not rejuvenate a pose when the lidar panel remounts', () => {
    const { getByRole } = render(<App />)
    act(() => {
      onMessage({ type: 'state', data: { slam } })
      onMessage({ type: 'scan', data: scan })
    })
    draw()
    expect(ctx.drawImage).toHaveBeenCalled()
    act(() => getByRole('button', { name: 'Свернуть панель «Лидар»' }).click())
    act(() => vi.advanceTimersByTime(650))
    // A normal state publication still carries the same result identity.
    act(() => onMessage({ type: 'state', data: { slam: { ...slam } } }))
    ctx.drawImage.mockClear()
    act(() => getByRole('button', { name: 'Открыть панель «Лидар»' }).click())
    draw()
    expect(ctx.drawImage).not.toHaveBeenCalled()
  })

  it('requires explicit matching IDs to globalize a legacy scan', () => {
    mount({ ...slam, scan_id: undefined }, 'map', { ...scan, id: undefined }); draw()
    expect(ctx.drawImage).toHaveBeenCalled()
    expect(ctx.arc).not.toHaveBeenCalled()
  })
})
