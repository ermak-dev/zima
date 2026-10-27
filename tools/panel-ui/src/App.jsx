import { useCallback, useEffect, useState } from 'react'
import usePanelSocket from './usePanelSocket.js'
import useMjpeg from './useMjpeg.js'
import useTeleop from './useTeleop.js'
import LidarCanvas from './LidarCanvas.jsx'

const fmt = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '—')
const pair = (range, voltage) => {
  const hasR = typeof range === 'number' && Number.isFinite(range)
  const hasV = typeof voltage === 'number' && Number.isFinite(voltage)
  if (!hasR && !hasV) return null
  return `${hasR ? range.toFixed(3) : '—'} м / ${hasV ? voltage.toFixed(2) : '—'} V`
}

function reading(text, cls) {
  return { text: text ?? 'нет данных', cls: text === null ? 'missing' : cls || '' }
}

function SensorRow({ id, name, value }) {
  return (
    <tr>
      <td>{name}</td>
      <td id={id} className={value.cls}>{value.text}</td>
    </tr>
  )
}

export default function App() {
  const [state, setState] = useState({})
  const [scan, setScan] = useState({})
  const [view, setView] = useState({ radius: 3, center: null })
  const [collapsed, setCollapsed] = useState({ lidar: false, camera: false, sensors: false })
  const [expanded, setExpanded] = useState(null)

  const onMessage = useCallback((msg) => {
    if (msg.type === 'state') setState(msg.data)
    else if (msg.type === 'scan') setScan(msg.data)
  }, [])
  const { connected, send } = usePanelSocket(onMessage)
  const { stop } = useTeleop(send)
  const cameraActive = !collapsed.camera && expanded !== 'lidar' && expanded !== 'sensors'
  const { status: camStatus, src: camSrc } = useMjpeg({ url: '/camera.mjpeg', active: cameraActive })

  // Odometry is telemetry only; the lidar view stays in the robot frame.
  const odom = state.odom || {}
  const sensors = state.sensors || {}
  const battery = sensors.battery
  const batteryKnown = battery && (Number.isFinite(battery.percent) || Number.isFinite(battery.volts) || typeof battery.charging === 'boolean')
  const batteryText = batteryKnown ? `${Number.isFinite(battery.percent) ? battery.percent : '—'}% / ${Number.isFinite(battery.volts) ? battery.volts : '—'} V${battery.charging === true ? ' · заряд' : ''}` : null

  const toggle = (id) => {
    setCollapsed((c) => ({ ...c, [id]: !c[id] }))
    if (expanded === id) setExpanded(null)
  }
  const toggleExpand = (id) => {
    setExpanded((e) => {
      const next = e === id ? null : id
      if (next) setCollapsed((c) => ({ ...c, [id]: false }))
      return next
    })
  }

  // Escape leaves the expanded layout and returns focus to its button.
  useEffect(() => {
    if (!expanded) return
    const onKey = (e) => {
      if (e.code !== 'Escape') return
      e.preventDefault()
      setExpanded(null)
      document.getElementById(`${expanded}-expand`)?.focus()
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [expanded])

  const panelBtn = (id, name) => (
    <>
      <button id={`${id}-collapse`} className="panel-action" type="button" aria-controls={`panel-${id}`}
        aria-expanded={!collapsed[id]}
        aria-label={`${collapsed[id] ? 'Открыть' : 'Свернуть'} панель «${name}»`}
        title={`${collapsed[id] ? 'Открыть' : 'Свернуть'} панель «${name}»`}
        onClick={() => toggle(id)}>
        {collapsed[id] ? '+' : '−'}
      </button>
      <button id={`${id}-expand`} className="panel-action" type="button" aria-controls={`panel-${id}`}
        aria-pressed={expanded === id}
        aria-label={`${expanded === id ? 'Вернуть размер' : 'Развернуть панель'} «${name}»`}
        title={`${expanded === id ? 'Вернуть размер' : 'Развернуть панель'} «${name}»`}
        onClick={() => toggleExpand(id)}>
        {expanded === id ? '⤡' : '⤢'}
      </button>
    </>
  )

  return (
    <>
      <header>
        <div className="brand">
          <svg viewBox="0 0 32 32" fill="none" aria-hidden="true"><circle cx="16" cy="16" r="13" stroke="currentColor" strokeWidth="1.5" /><circle cx="12" cy="16" r="4" stroke="currentColor" strokeWidth="1.5" /><path d="M22 10v12M25 13v6" stroke="currentColor" strokeWidth="1.5" /></svg>
          Roborock<span className="meta">V1</span><small>Рабочая панель</small>
        </div>
        <span id="connection" role="status">{connected ? 'Связь с роботом' : 'Нет связи с панелью'}</span>
      </header>
      <main id="workspace" data-expanded={expanded || ''}
        data-lidar-collapsed={collapsed.lidar} data-camera-collapsed={collapsed.camera} data-sensors-collapsed={collapsed.sensors}>
        <section id="panel-lidar" className={`lidar${collapsed.lidar ? ' is-collapsed' : ''}${expanded === 'lidar' ? ' is-expanded' : ''}`} aria-labelledby="lidar-title">
          <div className="heading">
            <h1 id="lidar-title">Лидар</h1>
            <div className="heading-end">
              <span className="meta lidar-meta" title="Колесо мыши меняет масштаб">
                <span className="zoom-hint">Колесо · </span><span id="scale-value">{fmt(view.radius, view.radius < 1 ? 2 : 1)} м</span>
                <span className="frame-label"> · <span id="frame-label">base_link</span></span>
              </span>
              <div className="panel-actions">
                <button id="lidar-center" type="button" title="Вернуть обзор к роботу" onClick={() => setView({ radius: 3, center: null })}>К роботу</button>
                {panelBtn('lidar', 'Лидар')}
              </div>
            </div>
          </div>
          {!collapsed.lidar && (
            <div className="lidar-stage">
              <LidarCanvas scan={scan} view={view} onView={setView} />
              <div className="legend">
                <span><i className="dot" />Скан лидара</span>
                <span><i className="robot-dot" />Робот · начало координат</span>
              </div>
            </div>
          )}
          <div className="pose" id="odom" aria-label="Одометрия">
            <span>x <b id="pose-x">{Number.isFinite(odom.x) ? `${fmt(odom.x)} м` : '—'}</b></span>
            <span>y <b id="pose-y">{Number.isFinite(odom.y) ? `${fmt(odom.y)} м` : '—'}</b></span>
            <span>θ <b id="pose-yaw">{Number.isFinite(odom.yaw) ? `${fmt(odom.yaw * 180 / Math.PI)} °` : '—'}</b></span>
            <span>v <b id="pose-v">{Number.isFinite(odom.vx) ? `${fmt(odom.vx)} м/с` : '—'}</b></span>
            <span>ω <b id="pose-w">{Number.isFinite(odom.yaw_rate) ? `${fmt(odom.yaw_rate)} рад/с` : '—'}</b></span>
          </div>
        </section>
        <aside>
          <section id="panel-camera" className={`camera${collapsed.camera ? ' is-collapsed' : ''}${expanded === 'camera' ? ' is-expanded' : ''}`} aria-labelledby="camera-title">
            <div className="heading">
              <h2 id="camera-title">Камера</h2>
              <div className="heading-end">
                <span id="cam-status">{camStatus === 'live' ? 'Прямой поток' : camStatus === 'off' ? 'Панель свёрнута' : camStatus === 'retrying' ? 'Нет потока' : 'Подключение'}</span>
                <div className="panel-actions">{panelBtn('camera', 'Камера')}</div>
              </div>
            </div>
            {!collapsed.camera && (
              <div className="camera-stage">
                <div id="cam-off" role="status" hidden={camStatus === 'live'}>
                  {camStatus === 'retrying' ? 'Камера недоступна. Повторное подключение…' : 'Подключение к камере…'}
                </div>
                {camStatus !== 'off' && (
                  <img id="cam" alt="Прямой поток с камеры робота" src={camSrc} />
                )}
              </div>
            )}
          </section>
          <section id="panel-sensors" className={`sensors${collapsed.sensors ? ' is-collapsed' : ''}${expanded === 'sensors' ? ' is-expanded' : ''}`} aria-labelledby="sensor-title">
            <div className="heading">
              <h2 id="sensor-title">Датчики шасси</h2>
              <div className="heading-end">
                <span className="meta sensor-meta">Range / V</span>
                <div className="panel-actions">{panelBtn('sensors', 'Датчики')}</div>
              </div>
            </div>
            {!collapsed.sensors && (
              <table className="sensor-table" aria-label="Все датчики шасси"><tbody>
                <tr><td>Батарея</td><td id="battery" className={batteryText === null ? 'missing' : ''}>{batteryText ?? 'нет данных'}</td></tr>
                <tr><td>Бампер</td><td id="bumper" className={sensors.bumper === 'PRESSED' ? 'alert' : sensors.bumper ? '' : 'missing'}>{sensors.bumper ? (sensors.bumper === 'PRESSED' ? 'Нажат' : 'Свободен') : 'нет данных'}</td></tr>
                <tr><td>Стена</td><td id="wall" className={pair(sensors.wall_ir, sensors.wall_voltages?.[0]) ? '' : 'missing'}>{pair(sensors.wall_ir, sensors.wall_voltages?.[0]) ?? 'нет данных'}</td></tr>
                <tr className="group"><td colSpan="2">Обрыв · расстояние / напряжение</td></tr>
                {[0, 1, 2, 3].map((i) => {
                  const v = pair(sensors.cliff_ranges?.[i], sensors.cliff_ir?.[i])
                  return <SensorRow key={i} id={`cliff-${i}`} name={`ИК ${i + 1}`} value={reading(v, '')} />
                })}
                <tr className="group"><td colSpan="2">Отрыв колёс · расстояние / напряжение</td></tr>
                {[0, 1].map((i) => {
                  const v = pair(sensors.drop_ranges?.[i], sensors.drop_ir?.[i])
                  return <SensorRow key={i} id={`drop-${i}`} name={`ИК ${i + 1}`} value={reading(v, '')} />
                })}
                <tr><td>Сонар</td><td id="sonar" className={Number.isFinite(sensors.sonar) ? (sensors.sonar > 65 ? 'alert' : '') : 'missing'}>
                  {Number.isFinite(sensors.sonar) ? (sensors.sonar > 65 ? '65.534 м · sentinel' : `${fmt(sensors.sonar, 3)} м`) : 'нет данных'}
                </td></tr>
              </tbody></table>
            )}
            {!collapsed.sensors && <p className="sensor-note">Нули ИК — сырые показания, не признак безопасности. 65.534 м сонара — вероятный sentinel, не дистанция.</p>}
          </section>
        </aside>
      </main>
      <footer>
        <span><kbd>W A S D</kbd> / <kbd>↑ ← ↓ →</kbd> удерживать</span>
        <button id="stop" type="button" onClick={stop}>Стоп · Space</button>
        <span className="limits">≤ 0.1 м/с &nbsp; ≤ 0.4 рад/с</span>
      </footer>
    </>
  )
}
