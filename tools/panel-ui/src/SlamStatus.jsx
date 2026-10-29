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
const count = (v) => Number.isInteger(v) && v >= 0 ? v : '—'

export default function SlamStatus({ slam, scan, slamState }) {
  const localState = useSlamState(slamState ? null : slam, scan)
  const freshness = slamState || localState
  if (!slam) return null
  const pose = slam.pose || {}
  const mode = slam.mode || 'lost'
  const d = slam.diagnostics || {}
  return (
    <div className="pose slam" id="slam" aria-label="SLAM">
      <span className="slam-mode" data-mode={mode}>
        SLAM · <b>{MODE_TEXT[mode] || mode}</b>
      </span>
      <span>трек <b>{TRACKING_TEXT[slam.tracking] || slam.tracking || '—'}</b></span>
      <span>x <b>{fmt(pose.x)}</b></span>
      <span>y <b>{fmt(pose.y)}</b></span>
      <span>θ <b>{fmt(Number.isFinite(pose.yaw) ? pose.yaw * 180 / Math.PI : undefined, 0)}°</b></span>
      <span>ICP <b>{d.accepted === true ? 'принято' : d.accepted === false ? 'отклонено' : '—'}</b></span>
      <span>причина <b>{d.reason || '—'}</b></span>
      {d.mapping_paused && <span>пополнение карты приостановлено · движение без компенсации</span>}
      <span>точки вход / ICP / опора <b>{count(d.input_points)} / {count(d.used_points)} / {count(d.reference_points)}</b></span>
      <span>соответствия <b>{count(d.correspondences)}</b> · порог <b>{fmt(d.correspondence_threshold_m, 3)} м</b></span>
      <span>goodness <b>{fmt(d.goodness)}</b></span>
      {mode !== 'mapping' && <span title="Прокси по разбросу частиц, не goodness ICP">MCL <b>{fmt(slam.localization_quality)}</b></span>}
      <span>итерации <b>{count(d.iterations)}</b></span>
      <span>ICP мс <b>{fmt(d.duration_ms, 1)}</b></span>
      <span>RMS м <b>{fmt(d.residual_rms, 3)}</b></span>
      <span>{freshness.stale ? 'устарело · ' : ''}{freshness.ageMs === null ? 'возраст неизвестен' : `возраст ≥ ${fmt(freshness.ageMs, 0)} мс / ${freshness.maxAgeMs} мс`}</span>
      {freshness.mismatch && <span>скан и поза не совпадают</span>}
      {slam.error && <span>ошибка <b>{slam.error}</b></span>}
      <span>{slam.source_time_verified === true ? 'время подтверждено' : 'время не подтверждено'}</span>
      <span>одометрия <b>{slam.odom_association || '—'}</b> · сдвиг <b>{fmt(slam.odom_skew_ms, 1)} мс</b></span>
      <span>пропущено <b>{count(slam.dropped_scans)}</b></span>
      {slam.deskew && <span title={slam.deskew.reason || ''}>deskew {slam.deskew.applied ? 'применён' : slam.deskew.requested ? 'недоступен' : 'выключен'}</span>}
    </div>
  )
}
