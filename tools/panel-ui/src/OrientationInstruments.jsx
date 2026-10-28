import { useId } from 'react'

const degrees = (v) => Number.isFinite(v) ? Number((v * 180 / Math.PI).toFixed(3)) : null
const angleText = (v) => v === null ? '—' : `${v.toFixed(1)}°`

export default function OrientationInstruments({ compass, gyro }) {
  const clip = useId()
  const yaw = degrees(compass?.yaw)
  const roll = degrees(gyro?.pos?.[3])
  const pitch = degrees(gyro?.pos?.[4])
  const gyroYaw = degrees(gyro?.pos?.[5])
  const heading = yaw === null ? null : (yaw % 360 + 360) % 360
  const attitudeKnown = roll !== null && pitch !== null
  return (
    <div className="orientation-instruments">
      <figure className="instrument">
        <h3>Компас</h3>
        <svg viewBox="0 0 120 120" role="img" aria-label="Компас" data-known={heading !== null}>
          <title>{heading === null ? 'Нет данных компаса' : `Курс датчика ${heading.toFixed(1)}°, не калиброванный север`}</title>
          <circle className="instrument-ring" cx="60" cy="60" r="44" />
          {Array.from({ length: 24 }, (_, i) => <path key={i} className="instrument-tick" d={i % 6 ? 'M60 18v3' : 'M60 16v7'} transform={`rotate(${i * 15} 60 60)`} />)}
          <text x="60" y="12" textAnchor="middle">0°</text>
          <text x="6" y="63" textAnchor="middle">90</text>
          <text x="60" y="116" textAnchor="middle">180</text>
          <text x="113" y="63" textAnchor="middle">270</text>
          <path className="instrument-axis" d="M35 60h50M60 35v50" />
          {heading !== null && <g data-compass-needle transform={`rotate(${-heading} 60 60)`}>
            <path className="instrument-pointer" d="M60 25l-7 36h14z" />
            <path className="instrument-tail" d="M60 86l-5-25h10z" />
            <circle className="instrument-center" cx="60" cy="60" r="4" />
          </g>}
        </svg>
        <figcaption>
          <b>{heading === null ? 'нет данных' : angleText(heading)}</b>
          <small>Угол датчика, не север</small>
        </figcaption>
      </figure>
      <figure className="instrument">
        <h3>Гироскоп</h3>
        <svg viewBox="0 0 120 120" role="img" aria-label="Гироскоп" data-known={attitudeKnown} data-roll={roll ?? ''} data-pitch={pitch ?? ''}>
          <title>{attitudeKnown ? `Крен ${angleText(roll)}, тангаж ${angleText(pitch)}; калибровка не подтверждена` : 'Нет данных гироскопа'}</title>
          <defs><clipPath id={clip}><circle cx="60" cy="60" r="43" /></clipPath></defs>
          <circle className="instrument-ring" cx="60" cy="60" r="44" />
          {attitudeKnown && <g clipPath={`url(#${clip})`}>
            <g transform={`rotate(${-roll} 60 60)`}>
              <g transform={`translate(0 ${Math.max(-45, Math.min(45, pitch)) * 0.6})`}>
                <path className="attitude-upper" d="M-60-60h240V60H-60z" />
                <path className="attitude-lower" d="M-60 60h240v180H-60z" />
                <path className="attitude-horizon" d="M-60 60h240" />
                <path className="instrument-tick" d="M45 48h30M50 36h20M45 72h30M50 84h20" />
              </g>
            </g>
          </g>}
          <path className="attitude-reference" d="M30 60h20l10 5 10-5h20" />
          <path className="instrument-pointer" d="M60 18l-4 7h8z" />
        </svg>
        <figcaption className="gyro-readings">
          <span title="Roll · position3d:0 pos[3]">Крен <b id="gyro-roll">{angleText(roll)}</b></span>
          <span title="Pitch · position3d:0 pos[4]">Тангаж <b id="gyro-pitch">{angleText(pitch)}</b></span>
          <span title="Yaw · position3d:0 pos[5]">Курс <b id="gyro-yaw">{angleText(gyroYaw)}</b></span>
        </figcaption>
      </figure>
    </div>
  )
}
