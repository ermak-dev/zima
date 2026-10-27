// SLAM verdict block: mode, map-frame pose and trust indicator fed by the
// server's state.slam. Renders nothing when SLAM is not running.
const MODE_TEXT = {
  mapping: 'построение карты',
  localize: 'локализация',
  lost: 'потеря локализации',
}

function fmt(v, digits = 2) {
  return Number.isFinite(v) ? v.toFixed(digits) : '—'
}

export default function SlamStatus({ slam }) {
  if (!slam) return null
  const pose = slam.pose || {}
  const mode = slam.mode || 'lost'
  const quality = Number.isFinite(slam.quality) ? slam.quality : null
  return (
    <div className="pose slam" id="slam" aria-label="SLAM">
      <span className="slam-mode" data-mode={mode}>
        SLAM · <b>{MODE_TEXT[mode] || mode}</b>
      </span>
      <span>x <b>{fmt(pose.x)}</b></span>
      <span>y <b>{fmt(pose.y)}</b></span>
      <span>θ <b>{fmt(Number.isFinite(pose.yaw) ? pose.yaw * 180 / Math.PI : undefined, 0)}°</b></span>
      <span>доверие <b>{quality === null ? '—' : quality.toFixed(2)}</b></span>
    </div>
  )
}
