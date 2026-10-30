import { LIMITS } from './useTeleop.js'

const value = (v, unit) => Number.isFinite(v) ? `${v.toFixed(2)} ${unit}` : '—'
const ratio = (v, limit) => Number.isFinite(v) ? Math.max(-1, Math.min(1, v / limit)) : null

function VelocityMeter({ y, label, amount, marker }) {
  return (
    <g>
      <text x="12" y={y + 5}>{label}</text>
      <rect className="odometry-track" x="24" y={y - 4} width="72" height="8" rx="4" />
      <path className="instrument-tick" d={`M60 ${y - 9}v18`} />
      {amount !== null && <rect className="odometry-meter" data-odometry-linear={marker === 'linear' ? '' : undefined}
        data-odometry-angular={marker === 'angular' ? '' : undefined} data-ratio={amount}
        x={amount < 0 ? 60 + amount * 36 : 60} y={y - 4} width={Math.abs(amount) * 36} height="8" rx="3" />}
      <text x="24" y={y + 20} textAnchor="middle">−</text>
      <text x="60" y={y + 20} textAnchor="middle">0</text>
      <text x="96" y={y + 20} textAnchor="middle">+</text>
    </g>
  )
}

export default function OdometryInstruments({ odom = {} }) {
  const positionKnown = Number.isFinite(odom.x) && Number.isFinite(odom.y)
  const extent = positionKnown ? Math.max(1, Math.ceil(Math.hypot(odom.x, odom.y))) : 1
  const x = positionKnown ? 60 + odom.x / extent * 34 : 60
  const y = positionKnown ? 60 - odom.y / extent * 34 : 60
  const yaw = Number.isFinite(odom.yaw) ? odom.yaw * 180 / Math.PI : NaN
  const linear = ratio(odom.vx, LIMITS.v)
  const angular = ratio(odom.yaw_rate, LIMITS.w)
  return (
    <div className="odometry-instruments" id="odom" role="group" aria-label="Одометрия">
      <figure className="instrument">
        <h3>Одометрия</h3>
        <svg viewBox="0 0 120 120" role="img" aria-label="Положение одометрии" data-known={positionKnown}>
          <title>{positionKnown ? `X ${value(odom.x, 'м')}, Y ${value(odom.y, 'м')}; радиус шкалы ${extent} м. Локальная система одометрии, не SLAM-карта.` : 'Нет данных положения одометрии'}</title>
          <circle className="instrument-ring" cx="60" cy="60" r="44" />
          <path className="instrument-axis" d="M16 60h88M60 16v88M38 22v76M82 22v76M22 38h76M22 82h76" />
          <path className="instrument-tick" d="M16 60h88M60 16v88" />
          <text x="109" y="63">X</text><text x="57" y="12">Y</text>
          <circle className="instrument-center" cx="60" cy="60" r="2" />
          {positionKnown && <g data-odometry-position data-x={odom.x} data-y={odom.y}>
            <path className="odometry-displacement" d={`M60 60L${x} ${y}`} />
            <circle className="instrument-pointer" cx={x} cy={y} r="4" />
            {Number.isFinite(yaw) && <path className="instrument-pointer" d={`M${x + 12} ${y}l-7-4v8z`} transform={`rotate(${-yaw} ${x} ${y})`} />}
          </g>}
        </svg>
        <figcaption className="gyro-readings">
          <span>X <b id="pose-x">{value(odom.x, 'м')}</b></span>
          <span>Y <b id="pose-y">{value(odom.y, 'м')}</b></span>
          <span>θ <b id="pose-yaw">{value(yaw, '°')}</b></span>
        </figcaption>
      </figure>
      <figure className="instrument">
        <h3>Скорость</h3>
        <svg viewBox="0 0 120 120" role="img" aria-label="Скорости одометрии" data-known={linear !== null || angular !== null}>
          <title>Измеренные скорости: v {value(odom.vx, 'м/с')}, ω {value(odom.yaw_rate, 'рад/с')}. Шкалы ±{LIMITS.v} м/с и ±{LIMITS.w} рад/с.</title>
          <VelocityMeter y={34} label="v" amount={linear} marker="linear" />
          <VelocityMeter y={82} label="ω" amount={angular} marker="angular" />
        </svg>
        <figcaption className="gyro-readings">
          <span>v <b id="pose-v">{value(odom.vx, 'м/с')}</b></span>
          <span>ω <b id="pose-w">{value(odom.yaw_rate, 'рад/с')}</b></span>
        </figcaption>
      </figure>
    </div>
  )
}
