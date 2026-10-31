import { useEffect, useState } from 'react'
import wsChannel from './wsChannel.js'

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
    const channel = wsChannel('/vision/ws', {
      onAttempt: () => { lastId = -1 },
      onMessage: (ws, event) => {
        if (!alive) return
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
          if (!alive || channel.socket !== ws || token !== generation) return
          img.onload = null
          img.onerror = null
          pending = null
          attempts = 0
          setValue({ status: 'live', frame: { ...data, src, objects: data.objects.filter(validObject) } })
        }
        img.onerror = () => {
          if (alive && channel.socket === ws && token === generation) clear('error', 'Не удалось декодировать кадр')
        }
        img.src = src
      },
      // Browsers follow a socket error with close; close also covers test/fake
      // transports that do not synthesize that event themselves.
      onError: (ws) => ws.close(),
      onClosed: () => {
        clear('disconnected')
        return Math.min(500 * 2 ** attempts++, 5000)
      },
    })
    return () => {
      alive = false
      clearTimeout(stale)
      cancelImage()
      channel.stop()
    }
  }, [active])
  return active ? value : empty('off')
}
