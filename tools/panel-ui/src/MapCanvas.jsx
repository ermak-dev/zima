import { useEffect, useRef } from 'react'

// The canvas is an imperative island: React never renders into it. The
// backing buffer is sized only when the element resizes (not on every
// frame), and redraws happen on data/interaction, not on a fixed timer.
export default function MapCanvas({
  occupancy, mapMeta, pose, scan, view, onView,
}) {
  const canvasRef = useRef(null)
  const stateRef = useRef({})
  stateRef.current = { occupancy, mapMeta, pose, scan, view, onView }
  const needsDraw = useRef(true)

  useEffect(() => { needsDraw.current = true }, [occupancy, mapMeta, pose, scan, view])

  useEffect(() => {
    const canvas = canvasRef.current
    const ctx = canvas.getContext('2d')

    const draw = () => {
      if (!needsDraw.current) return
      needsDraw.current = false
      const s = stateRef.current
      const box = canvas.getBoundingClientRect()
      const W = box.width, H = box.height
      if (!W || !H) return
      const dpr = window.devicePixelRatio || 1
      if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
        canvas.width = Math.round(W * dpr)
        canvas.height = Math.round(H * dpr)
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
      ctx.fillStyle = '#f8fafc'
      ctx.fillRect(0, 0, W, H)

      const frameMatch = !s.occupancy || (s.mapMeta && s.pose?.frame === s.occupancy.frame)
      const o = s.occupancy ? (frameMatch ? s.pose : null) : s.odomFallback
      const v = s.view
      const ox = v.center?.x ?? o?.x ?? 0
      const oy = v.center?.y ?? o?.y ?? 0
      const r = v.radius
      const k = Math.max(1, Math.min(W, H) - 54) / (2 * r)
      const at = (x, y) => [W / 2 + (x - ox) * k, H / 2 - (y - oy) * k]

      ctx.lineWidth = 1
      ctx.strokeStyle = '#e0e8ef'
      ctx.beginPath()
      const step = r <= 1 ? 0.25 : r <= 6 ? 1 : 5
      for (let x = Math.floor((ox - W / (2 * k)) / step) * step; x < ox + W / (2 * k); x += step) {
        const [p] = at(x, 0)
        ctx.moveTo(p, 0)
        ctx.lineTo(p, H)
      }
      for (let y = Math.floor((oy - H / (2 * k)) / step) * step; y < oy + H / (2 * k); y += step) {
        const [, p] = at(0, y)
        ctx.moveTo(0, p)
        ctx.lineTo(W, p)
      }
      ctx.stroke()
      ctx.fillStyle = '#576b7c'
      ctx.font = '11px system-ui'
      ctx.fillText(`${step} м`, W - 44, H - 12)

      if (s.occupancy && s.mapMeta?.bitmap) {
        const m = s.occupancy
        const [x, y] = at(m.origin.x, m.origin.y)
        ctx.save()
        ctx.translate(x, y)
        ctx.rotate(-m.origin.yaw)
        ctx.scale(m.resolution * k, -m.resolution * k)
        ctx.imageSmoothingEnabled = false
        ctx.drawImage(s.mapMeta.bitmap, 0, 0, m.width, m.height)
        ctx.restore()
      }
      if (s.showScan && (!s.occupancy || s.scan?.frame === s.occupancy.frame)) {
        ctx.fillStyle = '#2a78d6'
        for (const [x, y] of s.scan?.points || []) {
          const [a, b] = at(x, y)
          ctx.beginPath()
          ctx.arc(a, b, 2, 0, 2 * Math.PI)
          ctx.fill()
        }
      }
      if (o) {
        const [x, y] = at(o.x, o.y)
        const radius = 0.1725 * k
        ctx.save()
        ctx.translate(x, y)
        ctx.rotate(-o.yaw - Math.PI / 2)
        ctx.fillStyle = '#fff'
        ctx.strokeStyle = '#223446'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.arc(0, 0, radius, 0, 2 * Math.PI)
        ctx.fill()
        ctx.stroke()
        ctx.fillStyle = '#edf2f6'
        ctx.beginPath()
        ctx.arc(-0.1 * k, 0, 0.045 * k, 0, 2 * Math.PI)
        ctx.fill()
        ctx.stroke()
        ctx.beginPath()
        ctx.moveTo(radius + 10, 0)
        ctx.lineTo(radius + 3, -4)
        ctx.lineTo(radius + 3, 4)
        ctx.closePath()
        ctx.fillStyle = '#223446'
        ctx.fill()
        ctx.restore()
      }
      s.onRedraw?.()
    }

    const schedule = () => {
      needsDraw.current = true
      requestAnimationFrame(draw)
    }
    schedule()
    const observer = new ResizeObserver(schedule)
    observer.observe(canvas)
    // Pointer interactions: wheel zoom (bounded), drag pan.
    const onWheel = (e) => {
      e.preventDefault()
      if (!Number.isFinite(e.deltaY)) return
      const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? canvas.getBoundingClientRect().height : 1
      const radius = Math.max(0.35, Math.min(30, stateRef.current.view.radius * Math.exp(e.deltaY * unit * 0.0015)))
      stateRef.current.onView({ ...stateRef.current.view, radius })
      schedule()
    }
    let drag = null
    const onDown = (e) => {
      if (e.button !== 0) return
      const o = stateRef.current.occupancy ? stateRef.current.pose : stateRef.current.odomFallback
      const center = stateRef.current.view.center || { x: o?.x || 0, y: o?.y || 0 }
      drag = { id: e.pointerId, x: e.clientX, y: e.clientY, center }
      canvas.setPointerCapture?.(e.pointerId)
    }
    const onMove = (e) => {
      if (!drag || drag.id !== e.pointerId) return
      const { width, height } = canvas.getBoundingClientRect()
      const k = Math.max(1, Math.min(width, height) - 54) / (2 * stateRef.current.view.radius)
      const center = {
        x: drag.center.x - (e.clientX - drag.x) / k,
        y: drag.center.y + (e.clientY - drag.y) / k,
      }
      stateRef.current.onView({ ...stateRef.current.view, center })
    }
    const onUp = () => { drag = null }
    canvas.addEventListener('wheel', onWheel, { passive: false })
    canvas.addEventListener('pointerdown', onDown)
    canvas.addEventListener('pointermove', onMove)
    canvas.addEventListener('pointerup', onUp)
    canvas.addEventListener('pointercancel', onUp)
    return () => {
      observer.disconnect()
      canvas.removeEventListener('wheel', onWheel)
      canvas.removeEventListener('pointerdown', onDown)
      canvas.removeEventListener('pointermove', onMove)
      canvas.removeEventListener('pointerup', onUp)
      canvas.removeEventListener('pointercancel', onUp)
    }
  }, [])

  return <canvas id="scan" ref={canvasRef} style={{ touchAction: 'none', cursor: 'grab', display: 'block', width: '100%', height: '100%' }} aria-label="Карта занятости: светлое — свободно, тёмное — препятствия, серое — неизвестно. Перетаскивание — сдвиг, колесо — масштаб." />
}
