import { useEffect, useRef, useState } from 'react'

export const POLL_MS = 1000
export const DEADLINE_MS = 10 * 60 * 1000
export const UNCERTAIN = 'Исход неизвестен: выполнение или отмена не подтверждены.'
export const STATES = {
  queued: 'Речь в очереди',
  synthesizing: 'Синтез речи…',
  playing: 'Воспроизведение речи…',
  completed: 'Воспроизведение завершено',
  cancelled: 'Воспроизведение отменено',
  failed: 'Ошибка воспроизведения',
  unknown: UNCERTAIN,
  error: 'Ошибка аудиосервиса: исход неизвестен',
}
export const ACTIVE = new Set(['queued', 'synthesizing', 'playing'])
export const IDLE_STATUS = 'Речь · готово к вводу'

// TTS task lifecycle over the audio-only HTTP path: start, sequential 1 s
// polling, cancellation that only a poll can confirm, a 10 minute deadline and
// cleanup on unmount. Uncertain outcomes are never retried or reported as done.
export default function useSpeechTask() {
  const [busy, setBusy] = useState(false)
  const [taskId, setTaskId] = useState(null)
  const [cancelling, setCancelling] = useState(false)
  const [status, setStatus] = useState(IDLE_STATUS)
  const current = useRef(null)

  useEffect(() => () => {
    if (current.current) dispose(current.current)
    current.current = null
  }, [])

  const live = (run) => current.current === run && !run.controller.signal.aborted
  const finish = (run, message) => {
    if (!live(run)) return
    dispose(run)
    current.current = null
    setBusy(false)
    setTaskId(null)
    setCancelling(false)
    setStatus(message)
  }
  const request = async (run, url, options = {}, expected = 200) => {
    const res = await fetch(url, { ...options, signal: run.controller.signal })
    if (res.status !== expected) throw new Error('Unexpected HTTP response')
    return res.json()
  }
  const poll = async (run) => {
    try {
      const data = await request(run, `/api/audio/tasks/${encodeURIComponent(run.id)}`)
      if (!live(run)) return
      if (data.id !== run.id || !Object.hasOwn(STATES, data.state)) throw new Error('Invalid task')
      const message = STATES[data.state]
      if (!ACTIVE.has(data.state)) {
        finish(run, `${message}${typeof data.error === 'string' ? ` · ${data.error}` : ''}`)
        return
      }
      setStatus(run.cancelling ? 'Отмена запрошена · ожидаем подтверждения' : message)
      run.poll = setTimeout(() => poll(run), POLL_MS)
    } catch {
      finish(run, UNCERTAIN)
    }
  }
  const say = async (text, volume) => {
    if (current.current) return
    const run = { controller: new AbortController(), id: null, cancelling: false }
    current.current = run
    setBusy(true)
    setStatus('Отправка речи…')
    run.deadline = setTimeout(() => finish(run, `Нет подтверждения за 10 минут. ${UNCERTAIN}`), DEADLINE_MS)
    try {
      const data = await request(run, '/api/audio/say', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: text.trim(), volume }),
      }, 202)
      if (!live(run)) return
      if (typeof data.id !== 'string' || !data.id || data.state !== 'queued') throw new Error('Invalid acceptance')
      run.id = data.id
      setTaskId(data.id)
      setStatus(STATES.queued)
      run.poll = setTimeout(() => poll(run), POLL_MS)
    } catch {
      finish(run, UNCERTAIN)
    }
  }
  const cancel = async () => {
    const run = current.current
    if (!run?.id || run.cancelling) return
    run.cancelling = true
    setCancelling(true)
    setStatus('Отмена запрошена · ожидаем подтверждения')
    try {
      await request(run, `/api/audio/tasks/${encodeURIComponent(run.id)}/cancel`, { method: 'POST' }, 202)
      // Acknowledgement is not cancellation: only the task poll can confirm it.
    } catch {
      finish(run, UNCERTAIN)
    }
  }
  return { busy, taskId, cancelling, status, say, cancel }
}

function dispose(run) {
  clearTimeout(run.poll)
  clearTimeout(run.deadline)
  run.controller.abort()
}
