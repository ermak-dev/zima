// Top-down robot diagram with sensor indicators.
// Bumper segments, cliff/drop sensors, sonar and wall IR shown in place.
export default function RobotDiagram({ sensors = {} }) {
  const bumperSegments = Array.isArray(sensors.bumper_segments)
    ? sensors.bumper_segments
    : [undefined, undefined, undefined]
  const [leftBumper, frontBumper, rightBumper] = bumperSegments

  const cliffRanges = sensors.cliff_ranges || []
  const cliffIr = sensors.cliff_ir || []
  const dropRanges = sensors.drop_ranges || []
  const dropIr = sensors.drop_ir || []
  const wallKnown = Number.isFinite(sensors.wall_ir) || Number.isFinite(sensors.wall_voltages?.[0])
  const sonarKnown = Number.isFinite(sensors.sonar)
  const sonarNoEcho = sonarKnown && sensors.sonar > 65

  const cx = 100, cy = 100, r = 70

  const fmt = (v) => Number.isFinite(v) ? v.toFixed(2) : '—'

  // Bumper arc: startAngle to endAngle (degrees, 0 = front/up, CW positive)
  function bumperArc(startDeg, endDeg, state, name) {
    const pressed = state === true
    const unknown = typeof state !== 'boolean'
    const startRad = (startDeg - 90) * Math.PI / 180
    const endRad = (endDeg - 90) * Math.PI / 180
    const outerR = r + 10
    const innerR = r + 3
    const x1o = cx + outerR * Math.cos(startRad)
    const y1o = cy + outerR * Math.sin(startRad)
    const x2o = cx + outerR * Math.cos(endRad)
    const y2o = cy + outerR * Math.sin(endRad)
    const x2i = cx + innerR * Math.cos(endRad)
    const y2i = cy + innerR * Math.sin(endRad)
    const x1i = cx + innerR * Math.cos(startRad)
    const y1i = cy + innerR * Math.sin(startRad)
    const large = Math.abs(endDeg - startDeg) > 180 ? 1 : 0
    const d = `M${x1o},${y1o} A${outerR},${outerR} 0 ${large},1 ${x2o},${y2o}
               L${x2i},${y2i} A${innerR},${innerR} 0 ${large},0 ${x1i},${y1i} Z`
    const status = pressed ? 'нажат' : unknown ? 'нет данных' : 'свободен'
    return (
      <path
        d={d}
        className={`bumper-arc ${pressed ? 'pressed' : unknown ? 'unknown' : 'free'}`}
        aria-label={`Бампер ${name}: ${status}`}
      >
        <title>Бампер {name}: контактный датчик столкновения&#10;Состояние: {status}</title>
      </path>
    )
  }

  // Cliff sensor positions: front-left, front-right, rear-left, rear-right
  const cliffPositions = [
    { angle: -40, label: 'Обрыв 1' },
    { angle: 40, label: 'Обрыв 2' },
    { angle: -140, label: 'Обрыв 3' },
    { angle: 140, label: 'Обрыв 4' },
  ]

  // Drop sensors (wheels): left and right sides
  const dropPositions = [
    { angle: -90, label: 'Отрыв 1' },
    { angle: 90, label: 'Отрыв 2' },
  ]

  function sensorDot(angle, hasData, label, cls, description, value) {
    const rad = (angle - 90) * Math.PI / 180
    const dist = r - 8
    const x = cx + dist * Math.cos(rad)
    const y = cy + dist * Math.sin(rad)
    const tooltip = `${label}: ${description}\nЗначение: ${value}`
    return (
      <circle
        key={label}
        cx={x}
        cy={y}
        r={5}
        className={`sensor-dot ${cls} ${hasData ? 'received' : 'unknown'}`}
        aria-label={`${label}: ${hasData ? 'данные получены' : 'нет данных'}`}
      >
        <title>{tooltip}</title>
      </circle>
    )
  }

  return (
    <figure className="robot-diagram" aria-label="Диаграмма датчиков робота">
      <svg viewBox="0 0 200 200" role="img" aria-label="Робот сверху">
        {/* Robot body */}
        <circle cx={cx} cy={cy} r={r} className="robot-body" />

        {/* Direction indicator */}
        <polygon points={`${cx},${cy - r + 12} ${cx - 8},${cy - r + 26} ${cx + 8},${cy - r + 26}`} className="direction-arrow" />

        {/* Wheels (side bars) */}
        <rect x={cx - r - 4} y={cy - 20} width={8} height={40} rx={3} className="robot-wheel" />
        <rect x={cx + r - 4} y={cy - 20} width={8} height={40} rx={3} className="robot-wheel" />

        {/* Bumper arcs: left (-75 to -25), front (-25 to 25), right (25 to 75) */}
        {bumperArc(-75, -25, leftBumper, 'левый')}
        {bumperArc(-25, 25, frontBumper, 'передний')}
        {bumperArc(25, 75, rightBumper, 'правый')}

        {/* Sonar at front center */}
        <g className={`sonar-indicator ${sonarKnown ? (sonarNoEcho ? 'no-echo' : 'received') : 'unknown'}`}>
          <title>Сонар: ультразвуковой датчик расстояния{'\n'}Значение: {sonarKnown ? (sonarNoEcho ? '∞ (нет эха)' : `${sensors.sonar.toFixed(2)} м`) : 'нет данных'}</title>
          <circle cx={cx} cy={cy - r - 2} r={6} className="sonar-dot" />
          <text x={cx} y={cy - r - 11} textAnchor="middle" className="sonar-label">
            {sonarKnown ? (sonarNoEcho ? '∞' : `${sensors.sonar.toFixed(2)}`) : '?'}
          </text>
        </g>

        {/* Wall IR sensor (front-right area) */}
        <g className={`wall-ir-group ${wallKnown ? 'received' : 'unknown'}`}>
          <title>Стена: ИК датчик боковой стены{'\n'}Значение: {wallKnown ? `${fmt(sensors.wall_ir)} / ${fmt(sensors.wall_voltages?.[0])} В` : 'нет данных'}</title>
          <circle
            cx={cx + 25}
            cy={cy - r + 5}
            r={4}
            className={`wall-ir-dot ${wallKnown ? 'received' : 'unknown'}`}
          />
          <text x={cx + 25} y={cy - r - 3} textAnchor="middle" className="wall-label">W</text>
        </g>

        {/* Cliff sensors */}
        {cliffPositions.map((pos, i) => {
          const hasData = Number.isFinite(cliffRanges[i]) || Number.isFinite(cliffIr[i])
          const value = hasData ? `${fmt(cliffRanges[i])} м / ${fmt(cliffIr[i])} В` : 'нет данных'
          return sensorDot(pos.angle, hasData, pos.label, 'cliff', 'ИК датчик обрыва пола', value)
        })}

        {/* Drop sensors */}
        {dropPositions.map((pos, i) => {
          const hasData = Number.isFinite(dropRanges[i]) || Number.isFinite(dropIr[i])
          const value = hasData ? `${fmt(dropRanges[i])} м / ${fmt(dropIr[i])} В` : 'нет данных'
          return sensorDot(pos.angle, hasData, pos.label, 'drop', 'датчик отрыва колеса', value)
        })}

        {/* Center label */}
        <text x={cx} y={cy + 4} textAnchor="middle" className="robot-label">Zima</text>
      </svg>
      <figcaption className="robot-diagram-caption">
        <span className="legend-item bumper-legend free">Бампер свободен</span>
        <span className="legend-item bumper-legend pressed">Нажат</span>
        <span className="legend-item sensor-legend received">Датчик активен</span>
      </figcaption>
    </figure>
  )
}
