import { useEffect, useState } from 'react'

const empty = (status, error) => ({ status, error, frame: null })
const validObject = (object) => {
  const box = object?.box
  return typeof object?.label === 'string' && Number.isFinite(object.confidence)
    && object.confidence >= 0 && object.confidence <= 1
    && Array.isArray(box) && box.length === 4
    && box.every((n) => Number.isFinite(n) && n >= 0 && n <= 1)
    && box[0] < box[2] && box[1] < box[3]
}

// This channel is readonly and independent of drive /ws. A decoded JPEG and its
// detections are committed as one value; invalidated decodes can never revive it.
export default function useVision(active) {
  const [value, setValue] = useState(() => empty('off'))
  useEffect(() => {
    if (!active) { setValue(empty('off')); return }
    let alive = true
    let socket = null
    let retry = null
    let stale = null
    let pending = null
    let generation = 0
    let attempts = 0
    let lastId = -1

    const cancelImage = () => {
      generation += 1
      if (!pending) return
      pending.onload = null
      pending.onerror = null
      pending.src = ''
      pending = null
    }
    const clear = (status, error) => {
      clearTimeout(stale)
      cancelImage()
      setValue(empty(status, error))
    }
    const watchStale = () => {
      clearTimeout(stale)
      stale = setTimeout(() => clear('stale'), 3000)
    }
    const connect = () => {
      if (!alive) return
      lastId = -1
      clear('connecting')
      watchStale()
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      const ws = new WebSocket(`${proto}://${location.host}/vision/ws`)
      socket = ws
      ws.onmessage = (event) => {
        if (!alive || socket !== ws) return
        let message
        try { message = JSON.parse(event.data) } catch { return }
        const data = message?.data
        if (message?.type !== 'vision' || !data) return
        if (data.status === 'loading' || data.status === 'error') {
          clear(data.status, typeof data.error === 'string' ? data.error : undefined)
          return
        }
        if (data.status !== 'live' || !Number.isInteger(data.frame_id) || data.frame_id < 0
          || data.frame_id <= lastId || typeof data.jpeg !== 'string' || !data.jpeg
          || !Number.isFinite(data.width) || data.width <= 0
          || !Number.isFinite(data.height) || data.height <= 0 || !Array.isArray(data.objects)) return
        lastId = data.frame_id
        cancelImage()
        watchStale()
        const token = generation
        const src = `data:image/jpeg;base64,${data.jpeg}`
        const img = new Image()
        pending = img
        img.onload = () => {
          if (!alive || socket !== ws || token !== generation) return
          img.onload = null
          img.onerror = null
          pending = null
          attempts = 0
          setValue({ status: 'live', frame: { ...data, src, objects: data.objects.filter(validObject) } })
        }
        img.onerror = () => {
          if (alive && socket === ws && token === generation) clear('error', 'Не удалось декодировать кадр')
        }
        img.src = src
      }
      ws.onclose = () => {
        if (!alive || socket !== ws) return
        ws.onmessage = null
        ws.onclose = null
        ws.onerror = null
        socket = null
        clear('disconnected')
        retry = setTimeout(connect, Math.min(500 * 2 ** attempts++, 5000))
      }
      // Browsers follow a socket error with close; close also covers test/fake
      // transports that do not synthesize that event themselves.
      ws.onerror = () => ws.close()
    }
    connect()
    return () => {
      alive = false
      clearTimeout(retry)
      clearTimeout(stale)
      cancelImage()
      if (socket) {
        socket.onmessage = null
        socket.onclose = null
        socket.onerror = null
        socket.close()
      }
    }
  }, [active])
  return active ? value : empty('off')
}
