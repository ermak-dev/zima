// FEATURE: Plain velocity commands and keyboard stop safety.
// SPEC: No command on mount/view; key release, blur and hidden tabs stop.
// IMPLEMENTATION: Send plain {v,w}; process held-key releases regardless of focus.
// RESULT: IMPLEMENTED — stop paths clear held keys and the refresh interval.
// STATUS: DONE
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, renderHook } from '@testing-library/react'
import useTeleop from '../src/useTeleop.js'

afterEach(() => {
  cleanup()
  vi.useRealTimers()
  vi.restoreAllMocks()
})

const key = (type, code, target = window) => act(() => {
  target.dispatchEvent(new KeyboardEvent(type, { code, bubbles: true }))
})

describe('teleop safety', () => {
  it('is silent on mount, refreshes only held keys and stops on release', () => {
    vi.useFakeTimers()
    const send = vi.fn()
    const { unmount } = renderHook(() => useTeleop(send))
    act(() => vi.advanceTimersByTime(1000))
    expect(send).not.toHaveBeenCalled()
    key('keydown', 'KeyW')
    expect(send).toHaveBeenLastCalledWith({ v: 0.1, w: 0 })
    act(() => vi.advanceTimersByTime(150))
    expect(send).toHaveBeenCalledTimes(2)
    key('keyup', 'KeyW')
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
    const count = send.mock.calls.length
    act(() => vi.advanceTimersByTime(1000))
    expect(send).toHaveBeenCalledTimes(count)
    unmount()
    expect(vi.getTimerCount()).toBe(0)
  })

  it.each(['blur', 'visibilitychange'])('stops on %s and does not restart', (event) => {
    vi.useFakeTimers()
    const send = vi.fn()
    renderHook(() => useTeleop(send))
    key('keydown', 'KeyA')
    if (event === 'visibilitychange') vi.spyOn(document, 'hidden', 'get').mockReturnValue(true)
    act(() => (event === 'blur' ? window : document).dispatchEvent(new Event(event)))
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
    const count = send.mock.calls.length
    act(() => vi.advanceTimersByTime(1000))
    expect(send).toHaveBeenCalledTimes(count)
  })

  it('releases a held key even if focus moved to a control', () => {
    const send = vi.fn()
    renderHook(() => useTeleop(send))
    key('keydown', 'KeyW')
    const button = document.createElement('button')
    document.body.append(button)
    key('keyup', 'KeyW', button)
    button.remove()
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
  })

  it('ignores typing and stops with Space or the explicit stop action', () => {
    const send = vi.fn()
    const { result } = renderHook(() => useTeleop(send))
    const input = document.createElement('input')
    document.body.append(input)
    key('keydown', 'KeyW', input)
    input.remove()
    expect(send).not.toHaveBeenCalled()
    key('keydown', 'ArrowRight')
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: -0.4 })
    key('keydown', 'Space')
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
    key('keydown', 'KeyS')
    act(() => result.current.stop())
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
  })
})
