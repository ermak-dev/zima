import { useEffect, useRef } from 'react'

// Decode the adapter's CSV occupancy string ("50,-1,0,...") into per-cell
// occupancy percent; unknown cells (-1) become 255.
export function parseGrid(cells, w, h) {
  const out = new Uint8Array(w * h)
  if (typeof cells !== 'string') return out
  const parts = cells.split(',')
  const n = Math.min(parts.length, w * h)
  for (let i = 0; i < n; i++) {
    const v = Number(parts[i])
    out[i] = Number.isFinite(v) && v >= 0 ? Math.min(100, Math.max(0, v)) : 255
  }
  return out
}

const WALL_RGBA = [34, 52, 70, 235]
const FREE_RGBA = [42, 120, 214, 60]

// Color of one occupancy cell; null keeps it transparent (unknown or not
// yet decided either way).
export function cellRGBA(occ) {
  if (occ === 255) return null
  if (occ >= 65) return WALL_RGBA
  if (occ <= 35) return FREE_RGBA
  return null
}

// Canvas affine [a,b,c,d,e,f] taking grid image pixels (u right, v down,
// row 0 = highest y) through the map frame into base_link (inverse SLAM
// pose) and onto the screen used by `at()`.
export function gridTransform(grid, pose, { W, H, k, ox, oy }) {
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw)
  const res = grid.res
  const X0 = grid.x_min - pose.x
  const Y0 = grid.y_min + grid.h * res - pose.y
  return [
    k * c * res,
    k * s * res,
    -k * s * res,
    k * c * res,
    W / 2 - ox * k + k * (c * X0 + s * Y0),
    H / 2 + oy * k - k * (-s * X0 + c * Y0),
  ]
}

// Rasterize the grid once per distinct payload; redraws only re-project it.
function gridImage(cache, m) {
  if (cache.cells === m.cells && cache.w === m.w && cache.h === m.h) return cache.image
  cache.cells = m.cells
  cache.w = m.w
  cache.h = m.h
  cache.image = null
  const w = m.w | 0, h = m.h | 0
  const image = w && h ? document.createElement('canvas') : null
  const ictx = image?.getContext('2d')
  if (!ictx) return null
  image.width = w
  image.height = h
  const occ = parseGrid(m.cells, w, h)
  const data = ictx.createImageData(w, h)
  for (let j = 0; j < h; j++) {
    const row = (h - 1 - j) * w
    for (let i = 0; i < w; i++) {
      const rgba = cellRGBA(occ[j * w + i])
      if (rgba) data.data.set(rgba, (row + i) * 4)
    }
  }
  ictx.putImageData(data, 0, 0)
  cache.image = image
  return image
}

// The canvas is an imperative island: React never renders into it. The
// backing buffer is sized only when the element resizes (not on every
// frame), and redraws happen on data/interaction, not on a fixed timer.
export default function LidarCanvas({ scan, view, onView, slam }) {
  const canvasRef = useRef(null)
  const stateRef = useRef({})
  stateRef.current = { scan, view, onView, slam }
  const scheduleRef = useRef(null)
  const gridCacheRef = useRef({})

  useEffect(() => { scheduleRef.current?.() }, [scan, view, slam])

  useEffect(() => {
    const canvas = canvasRef.current
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    let frame = null

    const draw = () => {
      frame = null
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

      const v = s.view
      const ox = v.center?.x ?? 0
      const oy = v.center?.y ?? 0
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

      // SLAM occupancy grid under the live scan. The view is robot-centred,
      // so the map is projected through the inverse of the SLAM pose; without
      // a pose there is no consistent place to draw it.
      const map = s.slam?.map, pose = s.slam?.pose
      if (map?.cells && pose && [pose.x, pose.y, pose.yaw].every(Number.isFinite)) {
        const image = gridImage(gridCacheRef.current, map)
        if (image) {
          const [a, b, c, d, e, f] = gridTransform(map, pose, { W, H, k, ox, oy })
          ctx.save()
          ctx.setTransform(dpr * a, dpr * b, dpr * c, dpr * d, dpr * e, dpr * f)
          ctx.imageSmoothingEnabled = false
          ctx.drawImage(image, 0, 0)
          ctx.restore()
        }
      }

      // Points already include the static lidar mount correction.
      if (s.scan?.frame === 'base_link') {
        ctx.fillStyle = '#2a78d6'
        for (const [x, y] of s.scan?.points || []) {
          const [a, b] = at(x, y)
          ctx.beginPath()
          ctx.arc(a, b, 2, 0, 2 * Math.PI)
          ctx.fill()
        }
      }
      {
        const [x, y] = at(0, 0)
        const radius = 0.1725 * k
        ctx.save()
        ctx.translate(x, y)
        ctx.rotate(-Math.PI / 2)
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
    }

    const schedule = () => {
      if (frame === null) frame = requestAnimationFrame(draw)
    }
    scheduleRef.current = schedule
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
      const center = stateRef.current.view.center || { x: 0, y: 0 }
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
      scheduleRef.current = null
      if (frame !== null) cancelAnimationFrame(frame)
      observer.disconnect()
      canvas.removeEventListener('wheel', onWheel)
      canvas.removeEventListener('pointerdown', onDown)
      canvas.removeEventListener('pointermove', onMove)
      canvas.removeEventListener('pointerup', onUp)
      canvas.removeEventListener('pointercancel', onUp)
    }
  }, [])

  return <canvas id="scan" ref={canvasRef} style={{ touchAction: 'none', cursor: 'grab', display: 'block', width: '100%', height: '100%' }} aria-label="Текущий скан лидара относительно робота. X вправо, Y вверх. Перетаскивание — сдвиг, колесо — масштаб." />
}
