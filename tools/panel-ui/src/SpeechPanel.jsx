import { useEffect, useRef, useState } from 'react'

const POLL_MS = 1000
const DEADLINE_MS = 10 * 60 * 1000
const UNCERTAIN = 'Исход неизвестен: выполнение или отмена не подтверждены.'
const STATES = {
  queued: 'Речь в очереди',
  synthesizing: 'Синтез речи…',
  playing: 'Воспроизведение речи…',
  completed: 'Воспроизведение завершено',
  cancelled: 'Воспроизведение отменено',
  failed: 'Ошибка воспроизведения',
  unknown: UNCERTAIN,
  error: 'Ошибка аудиосервиса: исход неизвестен',
}
const ACTIVE = new Set(['queued', 'synthesizing', 'playing'])

function dispose(run) {
  clearTimeout(run.poll)
  clearTimeout(run.deadline)
  run.controller.abort()
}

export default function SpeechPanel({ settings } = {}) {
  const enabled = settings?.enabled !== false
  const maxVolume = Number.isFinite(settings?.max_volume) && settings.max_volume >= 0 && settings.max_volume <= 1 ? settings.max_volume : 0.9
  const initialVolume = Number.isFinite(settings?.volume) && settings.volume >= 0 && settings.volume <= maxVolume ? settings.volume : Math.min(0.35, maxVolume)
  const [text, setText] = useState('')
  const [volume, setVolume] = useState(initialVolume)
  const [busy, setBusy] = useState(false)
  const [taskId, setTaskId] = useState(null)
  const [cancelling, setCancelling] = useState(false)
  const [status, setStatus] = useState('Речь · готово к вводу')
  const current = useRef(null)
  const valid = text.trim().length > 0 && text.length <= 2000 && !text.includes('\0')

  useEffect(() => {
    if (!current.current) setVolume(initialVolume)
  }, [initialVolume, maxVolume])

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
  const say = async (event) => {
    event.preventDefault()
    if (!enabled || !valid || current.current || !Number.isFinite(volume) || volume < 0 || volume > maxVolume) return
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

  return (
    <form className="speech-panel" aria-label="Речь робота" noValidate onSubmit={say}>
      <label className="speech-text">Текст для озвучивания
        <textarea maxLength={2000} rows={2} value={text} onChange={(e) => setText(e.target.value)} />
      </label>
      <label className="speech-volume">Громкость речи
        <input type="range" min="0" max={maxVolume} step="0.01" value={volume}
          onChange={(e) => setVolume(Number(e.target.value))} />
      </label>
      <div className="speech-actions">
        <button type="submit" disabled={!enabled || !valid || busy}>Произнести</button>
        <button type="button" disabled={!taskId || cancelling} onClick={cancel}>Отменить речь</button>
      </div>
      <p role="status">{status}</p>
    </form>
  )
}
