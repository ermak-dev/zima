import { useEffect, useRef, useState } from 'react'
import useSpeechTask from './useSpeechTask.js'

export default function SpeechPanel({ settings } = {}) {
  const enabled = settings?.enabled !== false
  const maxVolume = Number.isFinite(settings?.max_volume) && settings.max_volume >= 0 && settings.max_volume <= 1 ? settings.max_volume : 0.9
  const initialVolume = Number.isFinite(settings?.volume) && settings.volume >= 0 && settings.volume <= maxVolume ? settings.volume : Math.min(0.35, maxVolume)
  const [text, setText] = useState('')
  const [volume, setVolume] = useState(initialVolume)
  const { busy, taskId, cancelling, status, say: sayTask, cancel } = useSpeechTask()
  const valid = text.trim().length > 0 && text.length <= 2000 && !text.includes('\0')

  useEffect(() => {
    if (!busy) setVolume(initialVolume)
  }, [initialVolume, maxVolume, busy])

  const say = (event) => {
    event.preventDefault()
    if (!enabled || !valid || busy || !Number.isFinite(volume) || volume < 0 || volume > maxVolume) return
    sayTask(text.trim(), volume)
  }

  const [open, setOpen] = useState(false)
  const dialogRef = useRef(null)

  const openModal = () => setOpen(true)
  const closeModal = () => setOpen(false)

  useEffect(() => {
    if (open) dialogRef.current?.showModal()
    else dialogRef.current?.close()
  }, [open])

  useEffect(() => {
    const dialog = dialogRef.current
    if (!dialog) return
    const onClose = () => setOpen(false)
    dialog.addEventListener('close', onClose)
    return () => dialog.removeEventListener('close', onClose)
  }, [])

  return (
    <>
      <button type="button" className="speech-trigger" onClick={openModal} aria-label="Открыть окно речи">
        <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M12 1a9 9 0 00-9 9v7c0 1.1.9 2 2 2h2v-9H5v-1a7 7 0 0114 0v1h-2v9h2c1.1 0 2-.9 2-2v-7a9 9 0 00-9-9z" fill="currentColor"/></svg>
        Речь
      </button>
      <dialog ref={dialogRef} className="speech-modal" aria-label="Речь робота">
        <form method="dialog" className="speech-panel" noValidate onSubmit={say}>
          <div className="speech-header">
            <h3>Речь робота</h3>
            <button type="button" className="speech-close" onClick={closeModal} aria-label="Закрыть">×</button>
          </div>
          <label className="speech-text">Текст для озвучивания
            <textarea maxLength={2000} rows={3} value={text} onChange={(e) => setText(e.target.value)} />
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
      </dialog>
    </>
  )
}
