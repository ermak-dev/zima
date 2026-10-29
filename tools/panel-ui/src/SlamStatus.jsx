import useSlamState from './useSlamState.js'

const MODE_TEXT = {
  mapping: 'построение карты',
  localize: 'локализация',
  lost: 'потеря локализации',
}
const TRACKING_TEXT = {
  initializing: 'инициализация', tracking: 'сопровождение', degraded: 'ухудшено',
  error: 'ошибка', lost: 'потеря локализации',
}
function fmt(v, digits = 2) {
  return Number.isFinite(v) ? v.toFixed(digits) : '—'
}

// User-facing SLAM status: mode, pose, freshness and honest failures.
// ICP internals, timing association and queue counters stay in the server
// state/diagnostics, not on the panel.
export default function SlamStatus({ slam, scan, slamState }) {
  const localState = useSlamState(slamState ? null : slam, scan)
  const freshness = slamState || localState
  if (!slam) return null
  const pose = slam.pose || {}
  const mode = slam.mode || 'lost'
  const paused = slam.diagnostics?.mapping_paused
  return (
    <div className="pose slam" id="slam" aria-label="SLAM">
      <span className="slam-mode" data-mode={mode}>
        SLAM · <b>{MODE_TEXT[mode] || mode}</b>
      </span>
      <span>трек <b>{TRACKING_TEXT[slam.tracking] || slam.tracking || '—'}</b></span>
      <span>x <b>{fmt(pose.x)}</b></span>
      <span>y <b>{fmt(pose.y)}</b></span>
      <span>θ <b>{fmt(Number.isFinite(pose.yaw) ? pose.yaw * 180 / Math.PI : undefined, 0)}°</b></span>
      {paused && <span>пополнение карты приостановлено · движение без компенсации</span>}
      <span>{freshness.stale ? 'устарело · ' : ''}{freshness.ageMs === null ? 'возраст неизвестен' : `возраст ≥ ${fmt(freshness.ageMs, 0)} мс / ${freshness.maxAgeMs} мс`}</span>
      {freshness.mismatch && <span>скан и поза не совпадают</span>}
      {slam.error && <span>ошибка <b>{slam.error}</b></span>}
    </div>
  )
}
