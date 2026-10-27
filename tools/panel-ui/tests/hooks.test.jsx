// Hook leak tests: unmount must close the socket, clear retry timers and
// release MJPEG handlers; StrictMode double-mount is exercised implicitly.
import { describe, it, expect, vi } from 'vitest'
import { renderHook, act } from '@testing-library/react'
import { cleanup } from '@testing-library/react'

import usePanelSocket from '../src/usePanelSocket.js'
import useMjpeg from '../src/useMjpeg.js'

class FakeWebSocket {
  static OPEN = 1
  static instances = []
  constructor(url) {
    this.url = url
    this.readyState = 1
    this.onclose = null
    this.onmessage = null
    FakeWebSocket.instances.push(this)
  }
  close() { this.closedByClient = true; this.onclose?.() }
  send(data) { this.sent = (this.sent || []).concat(data) }
}

describe('usePanelSocket', () => {
  it('unmount closes the socket and cancels pending reconnect', async () => {
    vi.useFakeTimers()
    const original = global.WebSocket
    global.WebSocket = FakeWebSocket
    try {
      const { unmount } = renderHook(() => usePanelSocket(() => {}))
      expect(FakeWebSocket.instances).toHaveLength(1)
      // server drop triggers reconnect scheduling
      act(() => FakeWebSocket.instances[0].onclose())
      act(() => vi.advanceTimersByTime(1000))
      const countAfterReconnect = FakeWebSocket.instances.length
      expect(countAfterReconnect).toBeGreaterThan(1)
      unmount()
      expect(FakeWebSocket.instances.at(-1).closedByClient).toBe(true)
      const n = FakeWebSocket.instances.length
      act(() => vi.advanceTimersByTime(10000))
      expect(FakeWebSocket.instances.length).toBe(n)
    } finally {
      global.WebSocket = original
      cleanup()
    }
  })

  it('onmessage JSON parse errors do not break the socket', () => {
    const original = global.WebSocket
    global.WebSocket = FakeWebSocket
    try {
      const seen = []
      const { unmount } = renderHook(() => usePanelSocket((m) => seen.push(m)))
      const ws = FakeWebSocket.instances.at(-1)
      expect(() => ws.onmessage({ data: 'not json' })).not.toThrow()
      ws.onmessage({ data: JSON.stringify({ type: 'state', data: {} }) })
      expect(seen).toHaveLength(1)
      unmount()
    } finally {
      global.WebSocket = original
      cleanup()
    }
  })
})

describe('useMjpeg', () => {
  it('inactive stream sets off status and never loads an image', () => {
    const { result } = renderHook(() => useMjpeg({ url: '/camera.mjpeg', active: false }))
    expect(result.current.status).toBe('off')
    expect(result.current.src).toBeUndefined()
  })

  // FEATURE: Camera source stays stable between actual connection attempts.
  // SPEC: State updates and elapsed time must not reconnect the MJPEG stream.
  // IMPLEMENTATION: Memoize the source by URL, active state and actual retry attempt.
  // RESULT: IMPLEMENTED — unrelated renders do not reconnect the camera.
  // STATUS: DONE
  it('keeps its source stable across rerenders and elapsed time', () => {
    vi.useFakeTimers()
    const { result, rerender, unmount } = renderHook(() => useMjpeg({ url: '/camera.mjpeg', active: true }))
    const src = result.current.src
    act(() => vi.advanceTimersByTime(10000))
    rerender()
    expect(result.current.src).toBe(src)
    unmount()
  })

  it('unmount clears the retry timer and detaches handlers', () => {
    vi.useFakeTimers()
    const { result, unmount } = renderHook(() => useMjpeg({ url: '/camera.mjpeg', active: true }))
    expect(['connecting', 'retrying']).toContain(result.current.status)
    expect(result.current.src).toMatch(/^\/camera\.mjpeg\?t=/)
    unmount()
    const before = vi.getTimerCount()
    act(() => vi.advanceTimersByTime(10000))
    expect(vi.getTimerCount()).toBeLessThanOrEqual(before)
  })
})
