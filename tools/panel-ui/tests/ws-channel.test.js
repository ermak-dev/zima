// Shared WebSocket lifecycle mechanics extracted from usePanelSocket and
// useVision (issue #52): URL building, connect, close-detach, consumer-driven
// backoff and full cleanup. Message handling stays in the consumers.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import wsChannel from '../src/wsChannel.js'

class FakeWebSocket {
  static OPEN = 1
  static instances = []
  constructor(url) {
    this.url = url
    this.readyState = 1
    this.sent = []
    FakeWebSocket.instances.push(this)
  }
  send(data) { this.sent.push(data) }
  close() { this.readyState = 3; this.onclose?.() }
  drop() { this.onclose?.() }
}

beforeEach(() => {
  vi.useFakeTimers()
  FakeWebSocket.instances = []
  vi.stubGlobal('WebSocket', FakeWebSocket)
})
afterEach(() => {
  vi.unstubAllGlobals()
  vi.useRealTimers()
})

describe('wsChannel', () => {
  it('connects to the ws(s) URL for the path and reports the current socket', () => {
    const channel = wsChannel('/ws', {})
    expect(FakeWebSocket.instances).toHaveLength(1)
    expect(channel.socket).toBe(FakeWebSocket.instances[0])
    expect(FakeWebSocket.instances[0].url).toBe(`ws://${location.host}/ws`)
    channel.stop()
  })

  it('wires open, message and closed callbacks to the live socket', () => {
    const opened = vi.fn()
    const seen = []
    const closed = vi.fn(() => 5000)
    const channel = wsChannel('/vision/ws', { onOpen: opened, onMessage: (ws, event) => seen.push([ws, event.data]), onClosed: closed })
    const ws = FakeWebSocket.instances[0]
    ws.onopen?.()
    expect(opened).toHaveBeenCalledWith(ws)
    ws.onmessage?.({ data: 'frame' })
    expect(seen).toEqual([[ws, 'frame']])
    ws.drop()
    expect(closed).toHaveBeenCalledWith(ws)
    expect(channel.socket).not.toBe(ws)
    channel.stop()
  })

  it('schedules the reconnect with the delay returned by onClosed', () => {
    let delay = 1500
    const channel = wsChannel('/ws', { onClosed: () => delay })
    expect(FakeWebSocket.instances).toHaveLength(1)
    FakeWebSocket.instances[0].drop()
    vi.advanceTimersByTime(1499)
    expect(FakeWebSocket.instances).toHaveLength(1)
    vi.advanceTimersByTime(1)
    expect(FakeWebSocket.instances).toHaveLength(2)
    channel.stop()
  })

  it('calls onAttempt before every connection attempt', () => {
    const attempts = vi.fn()
    const channel = wsChannel('/ws', { onAttempt: attempts, onClosed: () => 500 })
    expect(attempts).toHaveBeenCalledTimes(1)
    FakeWebSocket.instances[0].drop()
    vi.advanceTimersByTime(500)
    expect(attempts).toHaveBeenCalledTimes(2)
    channel.stop()
  })

  it('stop closes the live socket, detaches handlers and cancels a pending reconnect', () => {
    const channel = wsChannel('/ws', { onClosed: () => 5000 })
    const ws = FakeWebSocket.instances[0]
    ws.drop()
    channel.stop()
    const n = FakeWebSocket.instances.length
    vi.advanceTimersByTime(60_000)
    expect(FakeWebSocket.instances).toHaveLength(n)

    const live = wsChannel('/ws', {})
    const socket = FakeWebSocket.instances.at(-1)
    live.stop()
    expect(socket.readyState).toBe(3)
    expect(socket.onclose).toBeNull()
    expect(socket.onmessage).toBeNull()
    expect(live.socket).toBeNull()
  })

  it('stop detaches handlers so a late close event cannot schedule a reconnect', () => {
    const onClosed = vi.fn(() => 100)
    const channel = wsChannel('/ws', { onClosed })
    const ws = FakeWebSocket.instances[0]
    channel.stop()
    expect(ws.onclose).toBeNull()
    expect(ws.onmessage).toBeNull()
    ws.drop()
    vi.advanceTimersByTime(60_000)
    expect(FakeWebSocket.instances).toHaveLength(1)
    expect(onClosed).not.toHaveBeenCalled()
  })

  it('routes socket errors to onError for transports that need explicit close', () => {
    const errors = vi.fn((ws) => ws.close())
    const channel = wsChannel('/vision/ws', { onError: errors, onClosed: () => 1000 })
    const ws = FakeWebSocket.instances[0]
    ws.onerror?.()
    expect(errors).toHaveBeenCalledWith(ws)
    expect(ws.readyState).toBe(3)
    vi.advanceTimersByTime(1000)
    expect(FakeWebSocket.instances).toHaveLength(2)
    channel.stop()
  })

  it('does not reconnect when onClosed returns no delay', () => {
    const channel = wsChannel('/ws', { onClosed: () => undefined })
    FakeWebSocket.instances[0].drop()
    vi.advanceTimersByTime(60_000)
    expect(FakeWebSocket.instances).toHaveLength(1)
    channel.stop()
  })
})
