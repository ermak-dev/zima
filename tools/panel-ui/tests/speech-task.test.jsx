// Characterization tests for the TTS task lifecycle extracted from SpeechPanel
// (issue #52): 1 s sequential polling, a 10 minute deadline, honest uncertain
// outcomes and cancellation that only the task poll can confirm.
import { StrictMode } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, renderHook } from '@testing-library/react'
import useSpeechTask from '../src/useSpeechTask.js'

const response = (body, status = 200) => ({
  ok: status >= 200 && status < 300, status, json: async () => body,
})
const deferred = () => {
  let resolve
  const promise = new Promise((done) => { resolve = done })
  return { promise, resolve }
}
const tick = async (ms = 1000) => act(async () => { await vi.advanceTimersByTimeAsync(ms) })
const flush = async () => act(async () => {})
const posts = () => fetch.mock.calls.filter(([, options]) => options?.method === 'POST')
const polls = () => fetch.mock.calls.filter(([url]) => /^\/api\/audio\/tasks\/[^/]+$/.test(url))
const queued = () => fetch.mockResolvedValueOnce(response({ id: 'speech-1', state: 'queued' }, 202))
const task = (state, extra = {}) => response({ id: 'speech-1', state, ...extra })
const uncertain = () => /неизвест|неопредел|не удалось|нет связи|ошиб|не подтвержд/i
const say = async (hook, text = 'Привет, робот', volume = 0.35) => {
  await act(async () => { hook.result.current.say(text, volume) })
}

beforeEach(() => {
  vi.useFakeTimers()
  vi.stubGlobal('fetch', vi.fn().mockRejectedValue(new Error('Unexpected fetch')))
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
  vi.restoreAllMocks()
  vi.unstubAllGlobals()
})

describe('useSpeechTask', () => {
  it('is idle on mount with no requests, timers or task', async () => {
    const { result } = renderHook(() => useSpeechTask())
    expect(result.current.busy).toBe(false)
    expect(result.current.taskId).toBeNull()
    expect(result.current.cancelling).toBe(false)
    expect(result.current.status).toBe('Речь · готово к вводу')
    await tick(60_000)
    expect(fetch).not.toHaveBeenCalled()
    expect(vi.getTimerCount()).toBe(0)
  })

  it('submits trimmed JSON once and schedules the first poll after exactly 1 s', async () => {
    queued()
    fetch.mockResolvedValue(task('playing'))
    const hook = renderHook(() => useSpeechTask())
    await say(hook, '  Привет, робот  ')
    expect(posts()).toHaveLength(1)
    const [url, options] = posts()[0]
    expect(url).toBe('/api/audio/say')
    expect(JSON.parse(options.body)).toEqual({ text: 'Привет, робот', volume: 0.35 })
    expect(hook.result.current.busy).toBe(true)
    expect(hook.result.current.taskId).toBe('speech-1')
    expect(hook.result.current.status).toMatch(/очеред|синтез/i)
    await tick(999)
    expect(polls()).toHaveLength(0)
    await tick(1)
    expect(polls()).toHaveLength(1)
  })

  it('does not overlap a slow status request with the next poll', async () => {
    queued()
    const pending = deferred()
    fetch.mockReturnValueOnce(pending.promise)
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    await tick(5000)
    expect(polls()).toHaveLength(1)
    pending.resolve(task('playing'))
    fetch.mockResolvedValue(task('playing'))
    await flush()
  })

  it.each([
    ['completed', 'Воспроизведение завершено'],
    ['failed', 'Ошибка воспроизведения'],
    ['cancelled', 'Воспроизведение отменено'],
  ])('tracks states until terminal %s, then stops polling', async (state, message) => {
    queued()
    let current = 'synthesizing'
    fetch.mockImplementation(async () => task(current, state === 'failed' ? { error: 'Динамик недоступен' } : {}))
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    await tick()
    expect(hook.result.current.status).toMatch(/синтез/i)
    current = state
    await tick()
    expect(hook.result.current.status).toContain(message)
    expect(hook.result.current.busy).toBe(false)
    expect(hook.result.current.taskId).toBeNull()
    const count = fetch.mock.calls.length
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(count)
    expect(vi.getTimerCount()).toBe(0)
    expect(posts().filter(([url]) => url === '/api/audio/say')).toHaveLength(1)
  })

  it('reports the unknown task state as an uncertain outcome, not completion', async () => {
    queued()
    fetch.mockResolvedValue(task('unknown'))
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    await tick()
    expect(hook.result.current.status).toBe('Исход неизвестен: выполнение или отмена не подтверждены.')
    await tick(60_000)
    expect(vi.getTimerCount()).toBe(0)
  })

  it.each([
    ['acceptance', response({ id: 'speech-1', state: 'synthesizing' }, 202)],
    ['missing task id', response({ state: 'queued' }, 202)],
  ])('treats an invalid %s as an uncertain outcome', async (_name, acceptance) => {
    fetch.mockResolvedValueOnce(acceptance)
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    expect(hook.result.current.status).toBe('Исход неизвестен: выполнение или отмена не подтверждены.')
    expect(hook.result.current.busy).toBe(false)
    await tick(60_000)
    expect(vi.getTimerCount()).toBe(0)
  })

  it.each([
    ['id mismatch', task('playing', { id: 'другой' })],
    ['unknown state', response({ id: 'speech-1', state: 'exploded' })],
  ])('treats an invalid poll payload (%s) as an uncertain outcome', async (_name, payload) => {
    queued()
    fetch.mockResolvedValue(payload)
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    await tick()
    expect(hook.result.current.status).toBe('Исход неизвестен: выполнение или отмена не подтверждены.')
    await tick(60_000)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('requests cancellation once but only a poll can confirm it as cancelled', async () => {
    queued()
    let cancelled = false
    fetch.mockImplementation(async (url) => url.endsWith('/cancel')
      ? response({ id: 'speech-1', state: 'playing' }, 202)
      : task(cancelled ? 'cancelled' : 'playing'))
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    await tick()
    await act(async () => { hook.result.current.cancel() })
    await flush()
    expect(posts().filter(([url]) => url === '/api/audio/tasks/speech-1/cancel')).toHaveLength(1)
    expect(hook.result.current.cancelling).toBe(true)
    expect(hook.result.current.status).not.toMatch(/отменено/i)
    await act(async () => { hook.result.current.cancel() })
    await flush()
    expect(posts().filter(([url]) => url.endsWith('/cancel'))).toHaveLength(1)
    cancelled = true
    await tick()
    expect(hook.result.current.status).toMatch(/отменено/i)
    expect(hook.result.current.cancelling).toBe(false)
    expect(vi.getTimerCount()).toBe(0)
    expect(posts().filter(([url]) => url === '/api/audio/say')).toHaveLength(1)
  })

  it('bounds a hung submit with the 10 minute deadline without repeating it', async () => {
    fetch.mockReturnValue(new Promise(() => {}))
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    await tick(600_000 - 1)
    expect(hook.result.current.busy).toBe(true)
    await tick(1)
    expect(hook.result.current.busy).toBe(false)
    expect(hook.result.current.status).toContain('Нет подтверждения за 10 минут')
    expect(hook.result.current.status).toMatch(uncertain())
    expect(posts()).toHaveLength(1)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('bounds endless polling with the same 10 minute deadline', async () => {
    queued()
    fetch.mockResolvedValue(task('playing'))
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    await tick(600_000)
    expect(polls().length).toBeLessThan(601)
    expect(hook.result.current.status).toMatch(uncertain())
    expect(hook.result.current.status).not.toMatch(/завершено|отменено/i)
    const count = fetch.mock.calls.length
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(count)
    expect(vi.getTimerCount()).toBe(0)
  })

  it.each(['submit', 'poll', 'cancel'])('never retries or claims success after a %s network failure', async (phase) => {
    if (phase !== 'submit') queued()
    fetch.mockImplementation(async (url) => {
      if (phase === 'cancel' && !url.endsWith('/cancel')) return task('playing')
      throw new Error('Connection lost')
    })
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    if (phase !== 'submit') await tick()
    if (phase === 'cancel') { await act(async () => { hook.result.current.cancel() }); await flush() }
    expect(hook.result.current.status).toMatch(uncertain())
    await tick(60_000)
    expect(posts().filter(([url]) => url === '/api/audio/say')).toHaveLength(1)
    expect(hook.result.current.status).toMatch(uncertain())
  })

  it('treats an HTTP rejection as an uncertain outcome and stops', async () => {
    fetch.mockResolvedValue(response({ error: 'Аудиосервис недоступен' }, 503))
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    expect(hook.result.current.status).toMatch(uncertain())
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(1)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('ignores a second say while the original POST is unresolved', async () => {
    const pending = deferred()
    fetch.mockReturnValueOnce(pending.promise)
    const hook = renderHook(() => useSpeechTask())
    await say(hook)
    await say(hook)
    expect(posts()).toHaveLength(1)
    pending.resolve(response({ id: 'speech-1', state: 'queued' }, 202))
    await flush()
  })

  it.each(['submit', 'poll'])('releases timers, aborts requests and ignores a late %s response after unmount', async (phase) => {
    const pending = deferred()
    if (phase === 'poll') queued()
    fetch.mockReturnValue(pending.promise)
    const hook = renderHook(() => useSpeechTask(), { wrapper: StrictMode })
    await say(hook)
    if (phase === 'poll') await tick()
    const count = fetch.mock.calls.length
    const signals = fetch.mock.calls.map(([, options]) => options?.signal).filter(Boolean)
    hook.unmount()
    expect(vi.getTimerCount()).toBe(0)
    expect(signals.length).toBeGreaterThan(0)
    expect(signals.every((signal) => signal.aborted)).toBe(true)
    pending.resolve(phase === 'submit'
      ? response({ id: 'speech-1', state: 'queued' }, 202) : task('playing'))
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(count)
    expect(vi.getTimerCount()).toBe(0)
  })
})
