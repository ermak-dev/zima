import { useCallback, useEffect, useRef, useState } from 'react'
import usePanelSocket from './usePanelSocket.js'
import useMjpeg from './useMjpeg.js'
import useVision from './useVision.js'
import VisionFrame from './VisionFrame.jsx'
import useTeleop, { LIMITS as HARD_LIMITS, DEFAULT_LINEAR_SPEED, DEFAULT_ANGULAR_SPEED } from './useTeleop.js'
import LidarCanvas from './LidarCanvas.jsx'
import useSlamState from './useSlamState.js'
import { MotorsFooter } from './MotorsPanel.jsx'
import OrientationInstruments from './OrientationInstruments.jsx'
import useCompass from './useCompass.js'
import OdometryInstruments from './OdometryInstruments.jsx'
import RobotDiagram from './RobotDiagram.jsx'
import SpeechPanel from './SpeechPanel.jsx'
import zimaLogo from './assets/zima-logo.png?inline'

const fmt = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '—')

export default function App() {
  const [state, setState] = useState({})
  const [scan, setScan] = useState({})
  // The versioned SLAM grid arrives in its own `map` frames; the frequent
  // state carries only `slam.map_version`.
  const [mapFrame, setMapFrame] = useState(null)
  // Keep result age above collapsible panels: reopening a canvas is not a new result.
  const slamState = useSlamState(state.slam, scan)
  const [view, setView] = useState({ radius: 3, center: null, frame: 'robot', showScan: true, showMap: true })
  const [collapsed, setCollapsed] = useState({ lidar: false, camera: false, sensors: false })
  const [expanded, setExpanded] = useState(null)
  const [visionEnabled, setVisionEnabled] = useState(false)
  const [audioSettings, setAudioSettings] = useState(null)
  const [recoveryMessage, setRecoveryMessage] = useState('')
  const recoverySequence = useRef(0)

  useEffect(() => {
    const controller = new AbortController()
    fetch('/api/audio/config', { signal: controller.signal })
      .then((res) => res.ok ? res.json() : null)
      .then((settings) => {
        if (!controller.signal.aborted && settings && typeof settings.enabled === 'boolean' &&
          Number.isFinite(settings.volume) && Number.isFinite(settings.max_volume) &&
          settings.volume >= 0 && settings.volume <= settings.max_volume && settings.max_volume <= 1) {
          setAudioSettings(settings)
        }
      })
      .catch(() => {})
    return () => controller.abort()
  }, [])

  const onMessage = useCallback((msg) => {
    if (msg.type === 'state') setState(msg.data)
    else if (msg.type === 'scan') setScan(msg.data)
    else if (msg.type === 'map' && msg.data && Number.isInteger(msg.data.version)) setMapFrame(msg.data)
    else if (msg.type === 'vision' && typeof msg.data?.enabled === 'boolean') setVisionEnabled(msg.data.enabled)
    else if (msg.type === 'recovery') {
      setRecoveryMessage(msg.data?.isError
        ? (msg.data.error || 'Откат отказан')
        : 'Откат принят. После освобождения датчиков нажмите «Сброс».')
    }
  }, [])
  const { connected, send } = usePanelSocket(onMessage)
  const [speed, setSpeed] = useState(DEFAULT_LINEAR_SPEED)
  const linearLimit = Number.isFinite(speed) ? Math.min(Math.max(speed, 0.01), HARD_LIMITS.v) : DEFAULT_LINEAR_SPEED
  const [turnSpeed, setTurnSpeed] = useState(DEFAULT_ANGULAR_SPEED)
  const angularLimit = Number.isFinite(turnSpeed) ? Math.min(Math.max(turnSpeed, 0.01), HARD_LIMITS.w) : DEFAULT_ANGULAR_SPEED
  const blocked = state.blocked === true
  const { stop } = useTeleop(send, linearLimit, angularLimit, blocked)
  const cameraActive = !collapsed.camera && expanded !== 'lidar' && expanded !== 'sensors'
  const { status: rawStatus, src: camSrc } = useMjpeg({ url: '/camera.mjpeg', active: cameraActive && !visionEnabled })
  const vision = useVision(cameraActive && visionEnabled)
  const camStatus = visionEnabled ? vision.status : rawStatus
  const visionStatus = {
    off: 'Панель скрыта', connecting: 'Подключение', loading: 'Загрузка модели',
    live: 'Прямой поток', error: 'Ошибка', stale: 'Нет свежих кадров', disconnected: 'Нет связи · переподключение',
  }[vision.status]
  const camStatusText = visionEnabled
    ? `YOLO · ${visionStatus}${camStatus === 'live' && Number.isFinite(vision.frame?.inference_ms) ? ` · ${Math.round(vision.frame.inference_ms)} мс` : ''}`
    : camStatus === 'live' ? 'Прямой поток' : camStatus === 'off' ? 'Панель свёрнута' : camStatus === 'retrying' ? 'Нет потока' : 'Подключение'

  // Odometry is telemetry only; only an associated SLAM pose can globalize a scan.
  const odom = state.odom || {}
  const sensors = state.sensors || {}
  const compass = useCompass(sensors.compass, connected)
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
          <img className="brand-logo" src={zimaLogo} alt="Логотип Zima" width="28" height="28" />
          Zima<span className="meta">V1</span><small>Рабочая панель</small>
        </div>
        <span id="connection" role="status">{connected ? 'Связь с роботом' : 'Нет связи с панелью'}</span>
      </header>
      {blocked && <div className="blocked-balloon" role="alert" aria-label="Наезд на препятствие">
        <div>
          <strong>Наезд на препятствие</strong>
          <p>Обычное движение заблокировано. После бампера можно отъехать назад коротким импульсом, если позади безопасно. Обрыв/отрыв и устаревшие датчики запрещают откат.</p>
          <p>Один импульс: −0,1 м/с на 300 мс. Затем проверьте положение; после освобождения датчиков нажмите «Сброс».</p>
          {recoveryMessage && <p role="status">{recoveryMessage}</p>}
          {!connected && <p>Нет связи с сервером. Сброс и откат недоступны.</p>}
        </div>
        <div>
          <button type="button" disabled={!connected} onClick={() => {
            const command_id = `recover-${Date.now()}-${++recoverySequence.current}`
            setRecoveryMessage('Запрос отката…')
            send({ type: 'recover_bumper', command_id, duration_ms: 300 })
          }}>Отъехать назад</button>
          <button type="button" disabled={!connected} onClick={() => send({ type: 'reset_blocked' })}>Сброс</button>
        </div>
      </div>}
      <main id="workspace" data-expanded={expanded || ''}
        data-lidar-collapsed={collapsed.lidar} data-camera-collapsed={collapsed.camera} data-sensors-collapsed={collapsed.sensors}>
        <section id="panel-lidar" className={`lidar${collapsed.lidar ? ' is-collapsed' : ''}${expanded === 'lidar' ? ' is-expanded' : ''}`} aria-labelledby="lidar-title">
          <div className="heading">
            <h1 id="lidar-title">Лидар</h1>
            <div className="heading-end">
              <span className="meta lidar-meta" title="Колесо мыши меняет масштаб">
                <span className="zoom-hint">Колесо · </span><span id="scale-value">{fmt(view.radius, view.radius < 1 ? 2 : 1)} м</span>
                <span className="frame-label"> · <span id="frame-label">{view.frame === 'map' ? 'map' : 'base_link'}</span></span>
              </span>
              <div className="panel-actions">
                <label className="layer-toggle" title="Показывать точки текущего скана">
                  <input type="checkbox" checked={view.showScan} onChange={(e) => setView((v) => ({ ...v, showScan: e.target.checked }))} />Скан лидара
                </label>
                <label className="layer-toggle" title="Показывать карту SLAM (occupancy)">
                  <input type="checkbox" checked={view.showMap} onChange={(e) => setView((v) => ({ ...v, showMap: e.target.checked }))} />Карта SLAM
                </label>
                <button id="lidar-frame" type="button" aria-pressed={view.frame === 'map'}
                  title="Фиксированный кадр карты: отличить повреждение карты от скачка позы"
                  onClick={() => setView((v) => ({ ...v, frame: v.frame === 'map' ? 'robot' : 'map', center: null }))}>Кадр карты</button>
                <button id="lidar-center" type="button" title="Вернуть обзор к роботу" onClick={() => setView((v) => {
                  const p = state.slam?.pose
                  const center = v.frame === 'map' && Number.isFinite(p?.x) && Number.isFinite(p?.y) ? { x: p.x, y: p.y } : null
                  return { ...v, radius: 3, center }
                })}>К роботу</button>
                {panelBtn('lidar', 'Лидар')}
              </div>
            </div>
          </div>
          {!collapsed.lidar && (
            <div className="lidar-stage">
              <LidarCanvas scan={scan} view={view} onView={setView} slam={state.slam} mapFrame={mapFrame} slamState={slamState} />
              <div className="legend">
                <span><i className="dot" />Скан лидара</span>
                <span><i className="robot-dot" />{view.frame === 'map' ? 'Робот · поза SLAM' : 'Робот · начало координат'}</span>
                <span>Карта · занято / свободно / неизвестно</span>
                {view.frame === 'map' && <span>Фиксированный map · скан только при совпадении с позой</span>}
              </div>
            </div>
          )}
        </section>
        <aside>
          <section id="panel-camera" className={`camera${collapsed.camera ? ' is-collapsed' : ''}${expanded === 'camera' ? ' is-expanded' : ''}`} aria-labelledby="camera-title">
            <div className="heading">
              <h2 id="camera-title">Камера</h2>
              <div className="heading-end">
                <span id="cam-status">{camStatusText}</span>
                <div className="panel-actions">{panelBtn('camera', 'Камера')}</div>
              </div>
            </div>
            {!collapsed.camera && (
              <div className="camera-stage">
                <div id="cam-off" role="status" hidden={camStatus === 'live'}>
                  {visionEnabled ? (vision.error || camStatusText) : camStatus === 'retrying' ? 'Камера недоступна. Повторное подключение…' : 'Подключение к камере…'}
                </div>
                {visionEnabled && vision.frame && <VisionFrame frame={vision.frame} />}
                {!visionEnabled && camStatus !== 'off' && (
                  <img id="cam" alt="Прямой поток с камеры робота" src={camSrc} />
                )}
              </div>
            )}
          </section>
          <section id="panel-sensors" className={`sensors${collapsed.sensors ? ' is-collapsed' : ''}${expanded === 'sensors' ? ' is-expanded' : ''}`} aria-labelledby="sensor-title">
            <div className="heading">
              <h2 id="sensor-title">Датчики шасси</h2>
              <div className="heading-end">
                <div className="panel-actions">{panelBtn('sensors', 'Датчики')}</div>
              </div>
            </div>
            {!collapsed.sensors && (
              <div className="sensor-content">
                <div className="robot-panel">
                  <RobotDiagram sensors={sensors} />
                  <div className="status-badges">
                    <span id="battery" className={`status-badge ${batteryText === null ? 'unknown' : battery?.charging ? 'charging' : ''}`} aria-label="Батарея">
                      <svg className="badge-icon" viewBox="0 0 24 24" aria-hidden="true"><rect x="2" y="7" width="18" height="10" rx="2" fill="none" stroke="currentColor" strokeWidth="2"/><rect x="20" y="10" width="2" height="4" fill="currentColor"/><rect x="4" y="9" width="7" height="6" fill="currentColor" opacity=".6"/></svg>
                      <span className="badge-value">{batteryText ?? '—'}</span>
                    </span>
                    <span id="dock" className={`status-badge ${typeof sensors.dock !== 'boolean' ? 'unknown' : sensors.dock ? 'docked' : ''}`} aria-label="Док">
                      <svg className="badge-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M12 2v8M8 6l4 4 4-4M5 12v6a2 2 0 002 2h10a2 2 0 002-2v-6" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/></svg>
                      <span className="badge-value">{typeof sensors.dock === 'boolean' ? (sensors.dock ? 'На доке' : 'Вне дока') : '—'}</span>
                    </span>
                    <span id="bumper" className={`status-badge ${!sensors.bumper ? 'unknown' : sensors.bumper === 'PRESSED' ? 'alert' : ''}`} aria-label="Бампер общий">
                      <svg className="badge-icon" viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="12" r="9" fill="none" stroke="currentColor" strokeWidth="2"/><path d="M8 12h8M12 8v8" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/></svg>
                      <span className="badge-value">{sensors.bumper ? (sensors.bumper === 'PRESSED' ? 'Контакт' : 'Свободен') : '—'}</span>
                    </span>
                  </div>
                </div>
                <div className="instruments-panel">
                  <div className="instruments-grid">
                    <OrientationInstruments compass={compass} gyro={sensors.service_channels?.['position3d:0']} />
                    <OdometryInstruments odom={odom} />
                  </div>
                </div>
              </div>
            )}
          </section>
        </aside>
      </main>
      <footer>
        <span><kbd>W A S D</kbd> / <kbd>↑ ← ↓ →</kbd> удерживать</span>
        <label className="speed" htmlFor="speed-slider" title="Ограничение линейной скорости ручного управления">
          <span>Скорость <b id="speed-value">{`${linearLimit.toFixed(2)} м/с`}</b></span>
          <input id="speed-slider" type="range" min="0.01" max={HARD_LIMITS.v} step="0.01" value={linearLimit}
            onChange={(e) => setSpeed(Number(e.target.value))} />
        </label>
        <label className="speed" htmlFor="turn-speed-slider" title="Ограничение скорости поворота ручного управления">
          <span>Поворот <b id="turn-speed-value">{`${angularLimit.toFixed(2)} рад/с`}</b></span>
          <input id="turn-speed-slider" type="range" min="0.01" max={HARD_LIMITS.w} step="0.01" value={angularLimit}
            onChange={(e) => setTurnSpeed(Number(e.target.value))} />
        </label>
        <button id="stop" type="button" onClick={stop}>Стоп · Space</button>
        <span className="limits">≤ {HARD_LIMITS.v} м/с &nbsp; ≤ {HARD_LIMITS.w} рад/с</span>
        <MotorsFooter motors={state.motors} send={send} />
        <SpeechPanel settings={audioSettings} />
      </footer>
    </>
  )
}
