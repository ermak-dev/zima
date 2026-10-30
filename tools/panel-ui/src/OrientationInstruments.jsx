import { compassYaw } from './useCompass.js'

const degrees = (v) => {
  const result = typeof v === 'number' ? v * 180 / Math.PI : NaN
  return Number.isFinite(result) ? Number(result.toFixed(3)) : null
}
const angleText = (v) => v === null ? '—' : `${v.toFixed(1)}°`

const compassStatus = {
  connecting: 'Подключение датчика',
  stale: 'Нет свежих измерений',
  disconnected: 'Нет связи с сервером',
  unsupported: 'Обновите супервизор',
  unavailable: 'Датчик недоступен',
  invalid: 'Недостоверные измерения',
}

export default function OrientationInstruments({ compass, gyro }) {
  const yaw = degrees(compassYaw(compass))
  const gyroYaw = degrees(gyro?.pos?.[5])
  const heading = yaw === null ? null : (yaw % 360 + 360) % 360
  const compassNote = heading !== null ? '' : compassStatus[compass?.status] || 'Нет данных'
  return (
    <div className="orientation-instruments">
      <figure className="instrument">
        <h3>Компас</h3>
        <svg viewBox="0 0 120 120" role="img" aria-label="Компас" data-known={heading !== null}>
          <title>{heading === null ? compassNote : `Угол магнитного поля в осях датчика ${heading.toFixed(1)}°. Без калибровки; не курс робота и не север.`}</title>
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
          <small>{compassNote}</small>
        </figcaption>
      </figure>
      <figure className="instrument">
        <h3>Гироскоп</h3>
        <svg viewBox="0 0 120 120" role="img" aria-label="Гироскоп" data-known={gyroYaw !== null} data-yaw={gyroYaw ?? ''}>
          <title>{gyroYaw === null ? 'Нет данных курса гироскопа' : `Поворот в плоскости пола ${angleText(gyroYaw)}. Вид сверху; относительный курс, не магнитный север.`}</title>
          <circle className="instrument-ring" cx="60" cy="60" r="44" />
          <path className="instrument-axis" d="M16 60h88M60 16v88" />
          <text x="110" y="63" textAnchor="middle">0°</text>
          <text x="60" y="12" textAnchor="middle">90</text>
          <text x="9" y="63" textAnchor="middle">180</text>
          <text x="60" y="116" textAnchor="middle">270</text>
          {gyroYaw !== null && <g data-gyro-heading transform={`rotate(${-gyroYaw} 60 60)`}>
            <circle className="planar-robot" cx="60" cy="60" r="23" />
            <circle className="instrument-center" cx="49" cy="60" r="5" />
            <path className="planar-reference" d="M61 60h21" />
            <path className="instrument-pointer" d="M92 60l-11-6v12z" />
          </g>}
        </svg>
        <figcaption>
          <b id="gyro-yaw">{angleText(gyroYaw)}</b>
          <small>ω {Number.isFinite(gyro?.vel?.[5]) ? `${gyro.vel[5].toFixed(2)} рад/с` : '—'}</small>
        </figcaption>
      </figure>
    </div>
  )
}
