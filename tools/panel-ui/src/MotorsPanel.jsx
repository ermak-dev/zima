import { useEffect, useState } from 'react'

const MOTORS = [
  { key: 'fan', label: 'Вентилятор', hint: 'мощность 0–100 %' },
  { key: 'main', label: 'Основная щётка', hint: 'мощность 0–100 %' },
  { key: 'side', label: 'Боковая щётка', hint: 'мощность 0–100 %' },
]
const clamp = (v) => Math.max(0, Math.min(100, Math.round(v)))

// User-facing cleaning motor controls. The server owns safety: duties are
// clamped locally, ownership and stop-on-disconnect are enforced server-side.
export function MotorsFooter({ motors, send }) {
  return <MotorsPanel motors={motors} send={send} compact />
}

export default function MotorsPanel({ motors, send, compact = false }) {
  const known = motors && MOTORS.every(({ key }) => Number.isFinite(motors[key]))
  const [duties, setDuties] = useState(() =>
    Object.fromEntries(MOTORS.map(({ key }) => [key, known ? motors[key] : 0])))
  useEffect(() => {
    if (known) setDuties({ fan: motors.fan, main: motors.main, side: motors.side })
  }, [known, motors?.fan, motors?.main, motors?.side])

  const update = (key, value) => {
    const next = { ...duties, [key]: value }
    setDuties(next)
    send({ type: 'motors', fan: next.fan, main: next.main, side: next.side })
  }

  return (
    <div className={compact ? 'motors-footer' : 'pose'} id="motors" aria-label="Моторы уборки">
      {!compact && <span className="slam-mode">Моторы {known ? '' : '· состояние неизвестно'}</span>}
      {MOTORS.map(({ key, label, hint }) => (
        <label key={key} className="motor-control">
          <span>
            {label} <b>{known ? duties[key] : '—'}</b>
          </span>
          <input
            aria-label={`${label} (${hint})`}
            type="range"
            min="0"
            max="100"
            step="5"
            disabled={!send}
            value={duties[key]}
            onChange={(e) => update(key, clamp(Number(e.target.value)))}
          />
        </label>
      ))}
    </div>
  )
}
