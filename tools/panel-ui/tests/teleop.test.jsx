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

  // FEATURE: driver-ceiling linear teleop with an unchanged angular ceiling.
  // SPEC: allow 0.5 m/s, retain 0.4 rad/s, and reject nonfinite/oversized limits.
  // IMPLEMENTATION: shared UI limits with bounded linear and angular scaling.
  // RESULT: IMPLEMENTED — maximum speed, invalid limits and turn ceiling verified.
  // STATUS: DONE
  it.each([0.3, 0.5])('uses linear speed %s without increasing turn speed', (speed) => {
    const send = vi.fn()
    renderHook(() => useTeleop(send, speed))
    key('keydown', 'KeyW')
    expect(send).toHaveBeenLastCalledWith({ v: speed, w: 0 })
    key('keydown', 'KeyA')
    expect(send).toHaveBeenLastCalledWith({ v: speed, w: 0.4 })
    key('keyup', 'KeyW')
    key('keyup', 'KeyA')
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
  })

  it.each([1, Infinity, NaN, -1])('bounds the requested speed %s', (speed) => {
    const send = vi.fn()
    renderHook(() => useTeleop(send, speed))
    key('keydown', 'KeyW')
    key('keydown', 'KeyA')
    const command = send.mock.calls.at(-1)[0]
    expect(Number.isFinite(command.v)).toBe(true)
    expect(command.v).toBeGreaterThanOrEqual(0)
    expect(command.v).toBeLessThanOrEqual(0.5)
    expect(Number.isFinite(command.w)).toBe(true)
    expect(command.w).toBeGreaterThanOrEqual(0)
    expect(command.w).toBeLessThanOrEqual(0.4)
  })

  // FEATURE: independently adjustable angular speed.
  // SPEC: linear speed does not scale yaw; updated limits affect the held-key refresh.
  // IMPLEMENTATION: bound the angular argument independently and allow WASD on range inputs.
  // RESULT: IMPLEMENTED — independent limits, live refresh, signs and stop verified.
  // STATUS: DONE
  it('adjusts turn speed independently while keys are held', () => {
    vi.useFakeTimers()
    const send = vi.fn()
    const { rerender } = renderHook(({ v, w }) => useTeleop(send, v, w), {
      initialProps: { v: 0.06, w: 0.15 },
    })
    expect(send).not.toHaveBeenCalled()
    key('keydown', 'KeyW')
    key('keydown', 'KeyA')
    expect(send).toHaveBeenLastCalledWith({ v: 0.06, w: 0.15 })
    rerender({ v: 0.5, w: 0.25 })
    act(() => vi.advanceTimersByTime(150))
    expect(send).toHaveBeenLastCalledWith({ v: 0.5, w: 0.25 })
    key('keyup', 'KeyA')
    key('keydown', 'KeyD')
    expect(send).toHaveBeenLastCalledWith({ v: 0.5, w: -0.25 })
    key('keyup', 'KeyW')
    key('keyup', 'KeyD')
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
  })

  it.each([[1, 0.4], [-1, 0], [NaN, 0], [Infinity, 0]])('bounds angular limit %s to %s', (limit, expected) => {
    const send = vi.fn()
    renderHook(() => useTeleop(send, 0.1, limit))
    key('keydown', 'KeyA')
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: expected })
  })

  it('uses independent defaults for linear and angular speed', () => {
    const send = vi.fn()
    renderHook(() => useTeleop(send, 0.06))
    key('keydown', 'KeyW')
    expect(send).toHaveBeenLastCalledWith({ v: 0.06, w: 0 })
    key('keyup', 'KeyW')
    key('keydown', 'KeyA')
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0.4 })
    key('keyup', 'KeyA')
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
  })

  it('accepts WASD after a range adjustment but leaves arrows to the slider', () => {
    const send = vi.fn()
    renderHook(() => useTeleop(send, 0.1, 0.15))
    const slider = document.createElement('input')
    slider.type = 'range'
    document.body.append(slider)
    key('keydown', 'ArrowRight', slider)
    expect(send).not.toHaveBeenCalled()
    key('keydown', 'KeyA', slider)
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0.15 })
    key('keyup', 'KeyA', slider)
    expect(send).toHaveBeenLastCalledWith({ v: 0, w: 0 })
    slider.remove()
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
