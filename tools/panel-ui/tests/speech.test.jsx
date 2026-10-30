// FEATURE: Explicit Russian speech controls with bounded asynchronous task tracking.
// SPEC: POST /api/audio/say once with {text,volume}; default volume .35;
// trimmed nonempty text, no NUL, at most 2000 characters; UI volume in [0,.9].
// Poll task states without overlapping requests; queued is not completed;
// cancellation is not confirmed until the task says cancelled; uncertain outcomes
// never imply success or cause automatic re-POST. Release timers/requests on unmount.
// Polling safety policy: stop within ten minutes, including hung requests.
// IMPLEMENTATION: Explicit submit/cancel with sequential polling, abort cleanup and a shared deadline.
// RESULT: IMPLEMENTED — task outcomes remain distinct from request acknowledgements.
// STATUS: DONE
import { StrictMode } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import SpeechPanel from '../src/SpeechPanel.jsx'
import useTeleop from '../src/useTeleop.js'
import App from '../src/App.jsx'

vi.mock('../src/usePanelSocket.js', () => ({ default: () => ({ connected: false, send: () => {} }) }))
vi.mock('../src/useMjpeg.js', () => ({ default: () => ({ status: 'off' }) }))
vi.mock('../src/useVision.js', () => ({ default: () => ({ status: 'off' }) }))
vi.mock('../src/LidarCanvas.jsx', () => ({ default: () => null }))

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
const openModal = () => {
  const trigger = screen.getByRole('button', { name: 'Открыть окно речи' })
  fireEvent.click(trigger)
}
const input = () => screen.getByRole('textbox', { name: 'Текст для озвучивания' })
const submit = () => screen.getByRole('button', { name: 'Произнести' })
const cancel = () => screen.getByRole('button', { name: 'Отменить речь' })
const statusText = () => screen.getByRole('status').textContent
const posts = () => fetch.mock.calls.filter(([, options]) => options?.method === 'POST')
const polls = () => fetch.mock.calls.filter(([url]) => /^\/api\/audio\/tasks\/[^/]+$/.test(url))
const enter = (text) => fireEvent.change(input(), { target: { value: text } })
const say = async (text = 'Привет, робот') => {
  enter(text)
  fireEvent.click(submit())
  await flush()
}
const queued = () => fetch.mockResolvedValueOnce(response({ id: 'speech-1', state: 'queued' }, 202))
const task = (state, extra = {}) => response({ id: 'speech-1', state, ...extra })
const expectUncertain = () => {
  expect(statusText()).toMatch(/неизвест|неопредел|не удалось|нет связи|ошиб|не подтвержд/i)
  expect(statusText()).not.toMatch(/завершено|успешно|отменено/i)
}

beforeEach(() => {
  vi.useFakeTimers()
  vi.stubGlobal('fetch', vi.fn().mockRejectedValue(new Error('Unexpected fetch')))
  // Mock HTMLDialogElement methods for jsdom
  HTMLDialogElement.prototype.showModal = vi.fn(function() { this.open = true })
  HTMLDialogElement.prototype.close = vi.fn(function() { this.open = false })
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
  vi.restoreAllMocks()
  vi.unstubAllGlobals()
})

describe('speech panel', () => {
  it('uses shared server volume defaults and limits without receiving keys', async () => {
    queued()
    fetch.mockResolvedValue(task('playing'))
    render(<SpeechPanel settings={{ enabled: true, volume: 0.45, max_volume: 0.6 }} />)
    openModal()
    const slider = screen.getByRole('slider', { name: 'Громкость речи' })
    expect(Number(slider.value)).toBe(0.45)
    expect(Number(slider.max)).toBe(0.6)
    await say()
    expect(JSON.parse(posts()[0][1].body).volume).toBe(0.45)
  })

  it('does not submit when server configuration disables speech', async () => {
    render(<SpeechPanel settings={{ enabled: false, volume: 0.35, max_volume: 0.9 }} />)
    openModal()
    enter('Привет')
    expect(submit().disabled).toBe(true)
    fireEvent.click(submit())
    await flush()
    expect(fetch).not.toHaveBeenCalled()
  })
  it('loads only public settings on App mount without starting speech', async () => {
    fetch.mockResolvedValueOnce(response({ enabled: true, volume: 0.45, max_volume: 0.6 }))
    render(<App />)
    await flush()
    openModal()
    expect(screen.getByRole('dialog', { name: 'Речь робота' })).toBeTruthy()
    expect(input()).not.toBeNull()
    expect(fetch).toHaveBeenCalledTimes(1)
    expect(fetch.mock.calls[0][0]).toBe('/api/audio/config')
    expect(posts()).toHaveLength(0)
    expect(Number(screen.getByRole('slider', { name: 'Громкость речи' }).value)).toBe(0.45)
  })

  it('is idle on mount and exposes labelled Russian controls with safe defaults', async () => {
    render(<SpeechPanel />)
    openModal()
    expect(input().value).toBe('')
    expect(input().maxLength).toBe(2000)
    const volume = screen.getByRole('slider', { name: 'Громкость речи' })
    expect(Number(volume.value)).toBe(0.35)
    expect(Number(volume.min)).toBe(0)
    expect(Number(volume.max)).toBeLessThanOrEqual(1)
    expect(Number(volume.max)).toBeGreaterThanOrEqual(0.35)
    expect(submit().disabled).toBe(true)
    expect(screen.queryByRole('button', { name: 'Отменить речь' })?.disabled ?? true).toBe(true)
    await tick(60_000)
    expect(fetch).not.toHaveBeenCalled()
    expect(vi.getTimerCount()).toBe(0)
  })

  it('submits trimmed text once as JSON without browser keys, and does not claim playback success on 202', async () => {
    queued()
    fetch.mockResolvedValue(task('synthesizing'))
    render(<SpeechPanel />)
    openModal()
    await say('  Привет, робот  ')
    expect(posts()).toHaveLength(1)
    const [url, options] = posts()[0]
    expect(url).toBe('/api/audio/say')
    expect(JSON.parse(options.body)).toEqual({ text: 'Привет, робот', volume: 0.35 })
    expect(new Headers(options.headers).get('Content-Type')).toBe('application/json')
    expect([...new Headers(options.headers).keys()]).not.toContain('authorization')
    expect(statusText()).toMatch(/очеред|синтез|подготов/i)
    expect(statusText()).not.toMatch(/завершено|успешно/i)
    expect(submit().disabled).toBe(true)
    expect(cancel().disabled).toBe(false)
  })

  it.each(['', '   \n\t', 'до\0после', 'я'.repeat(2001)])('rejects invalid speech text case %# without a request', async (text) => {
    render(<SpeechPanel />)
    openModal()
    enter(text)
    fireEvent.click(submit())
    await flush()
    expect(fetch).not.toHaveBeenCalled()
  })

  it.each(['я', 'я'.repeat(2000)])('accepts the text length boundary case %#', async (text) => {
    queued()
    fetch.mockResolvedValue(task('playing'))
    render(<SpeechPanel />)
    openModal()
    await say(text)
    expect(posts()).toHaveLength(1)
    expect(JSON.parse(posts()[0][1].body).text).toBe(text)
  })

  it.each([0, 0.6, 0.9])('sends explicitly selected speech volume %s without changing any motion limit', async (volume) => {
    queued()
    fetch.mockResolvedValue(task('playing'))
    render(<SpeechPanel />)
    openModal()
    fireEvent.change(screen.getByRole('slider', { name: 'Громкость речи' }), { target: { value: volume } })
    await say()
    expect(JSON.parse(posts()[0][1].body)).toEqual({ text: 'Привет, робот', volume })
  })

  it('does not trigger WASD/arrow movement while entering speech', () => {
    const send = vi.fn()
    function Harness() { useTeleop(send); return <SpeechPanel /> }
    render(<Harness />)
    openModal()
    for (const code of ['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']) {
      fireEvent.keyDown(input(), { code })
      fireEvent.keyUp(input(), { code })
    }
    enter('wasd — текст, не движение')
    expect(send).not.toHaveBeenCalled()
    expect(fetch).not.toHaveBeenCalled()
  })

  it('blocks duplicate submits while the original POST is unresolved', async () => {
    const pending = deferred()
    fetch.mockReturnValueOnce(pending.promise)
    render(<SpeechPanel />)
    openModal()
    await say()
    fireEvent.click(submit())
    expect(posts()).toHaveLength(1)
    expect(submit().disabled).toBe(true)
    pending.resolve(response({ id: 'speech-1', state: 'queued' }, 202))
    await flush()
  })

  it('tracks synthesis and playback until completed, then stops polling', async () => {
    queued()
    let state = 'synthesizing'
    fetch.mockImplementation(async () => task(state))
    render(<SpeechPanel />)
    openModal()
    await say()
    await tick()
    expect(polls().length).toBeGreaterThan(0)
    expect(statusText()).toMatch(/синтез|подготов/i)
    state = 'playing'
    await tick()
    expect(statusText()).toMatch(/воспроиз|звуч|проигры/i)
    state = 'completed'
    await tick()
    expect(statusText()).toMatch(/завершено/i)
    expect(submit().disabled).toBe(false)
    const count = fetch.mock.calls.length
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(count)
    expect(vi.getTimerCount()).toBe(0)
    expect(posts()).toHaveLength(1)
  })

  it.each(['failed', 'cancelled', 'unknown'])('shows terminal %s honestly and stops polling', async (state) => {
    queued()
    fetch.mockResolvedValue(task(state, { error: 'Динамик недоступен <script>bad()</script>' }))
    render(<SpeechPanel />)
    openModal()
    await say()
    await tick()
    if (state === 'failed') expect(statusText()).toContain('Динамик недоступен')
    if (state === 'cancelled') expect(statusText()).toMatch(/отменено/i)
    if (state === 'unknown') expectUncertain()
    expect(document.querySelector('script')).toBeNull()
    const count = fetch.mock.calls.length
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(count)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('requests cancellation for the exact task but waits for confirmation before saying cancelled', async () => {
    queued()
    let cancelled = false
    fetch.mockImplementation(async (url) => url.endsWith('/cancel')
      ? response({ id: 'speech-1', state: 'playing' }, 202)
      : task(cancelled ? 'cancelled' : 'playing'))
    render(<SpeechPanel />)
    openModal()
    await say()
    await tick()
    fireEvent.click(cancel())
    await flush()
    expect(posts().filter(([url]) => url === '/api/audio/tasks/speech-1/cancel')).toHaveLength(1)
    expect(statusText()).not.toMatch(/отменено/i)
    cancelled = true
    await tick()
    expect(statusText()).toMatch(/отменено/i)
    expect(posts().filter(([url]) => url === '/api/audio/say')).toHaveLength(1)
    expect(vi.getTimerCount()).toBe(0)
  })

  it.each(['submit', 'poll', 'cancel'])('does not retry speech or report success after a %s network failure', async (phase) => {
    if (phase !== 'submit') queued()
    fetch.mockImplementation(async (url) => {
      if (phase === 'cancel' && !url.endsWith('/cancel')) return task('playing')
      throw new Error('Connection lost')
    })
    render(<SpeechPanel />)
    openModal()
    await say()
    if (phase !== 'submit') await tick()
    if (phase === 'cancel') { fireEvent.click(cancel()); await flush() }
    expectUncertain()
    await tick(60_000)
    expect(posts().filter(([url]) => url === '/api/audio/say')).toHaveLength(1)
    expectUncertain()
  })

  it('reports an HTTP rejection without treating it as queued or completed', async () => {
    fetch.mockResolvedValue(response({ error: 'Аудиосервис недоступен' }, 503))
    render(<SpeechPanel />)
    openModal()
    await say()
    expectUncertain()
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(1)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('does not overlap slow status requests and bounds active polling even if completion never arrives', async () => {
    queued()
    const pending = deferred()
    fetch.mockReturnValueOnce(pending.promise)
    render(<SpeechPanel />)
    openModal()
    await say()
    await tick(5000)
    expect(polls()).toHaveLength(1)
    pending.resolve(task('playing'))
    fetch.mockResolvedValue(task('playing'))
    await flush()
    await tick(600_000)
    const count = fetch.mock.calls.length
    expect(polls().length).toBeLessThan(601)
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(count)
    expect(vi.getTimerCount()).toBe(0)
    expectUncertain()
  })

  it('bounds a hung POST without automatically repeating it', async () => {
    fetch.mockReturnValue(new Promise(() => {}))
    render(<SpeechPanel />)
    openModal()
    await say()
    await tick(600_000)
    expect(posts()).toHaveLength(1)
    expectUncertain()
    expect(vi.getTimerCount()).toBe(0)
  })

  it.each(['submit', 'poll'])('releases timers and ignores a late %s response after StrictMode unmount', async (phase) => {
    const pending = deferred()
    if (phase === 'poll') queued()
    fetch.mockReturnValue(pending.promise)
    const { unmount } = render(<StrictMode><SpeechPanel /></StrictMode>)
    openModal()
    await say()
    if (phase === 'poll') await tick()
    const count = fetch.mock.calls.length
    const signals = fetch.mock.calls.map(([, options]) => options?.signal).filter(Boolean)
    unmount()
    expect(vi.getTimerCount()).toBe(0)
    expect(signals.length).toBeGreaterThan(0)
    expect(signals.every((signal) => signal.aborted)).toBe(true)
    pending.resolve(phase === 'submit'
      ? response({ id: 'speech-1', state: 'queued' }, 202) : task('playing'))
    await flush()
    await tick(60_000)
    expect(fetch).toHaveBeenCalledTimes(count)
    expect(vi.getTimerCount()).toBe(0)
  })
})
