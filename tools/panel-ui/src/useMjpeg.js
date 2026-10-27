import { useEffect, useRef, useState } from 'react'

// MJPEG in an <img> decodes every frame even when hidden, so the stream is
// paused (src released) while the panel is collapsed and fully stopped on
// unmount. The retry timer is cleared on every path.
export default function useMjpeg({ url, active }) {
  const [status, setStatus] = useState('connecting')
  const [attempt, setAttempt] = useState(0)

  useEffect(() => {
    if (!active) {
      setStatus('off')
      return
    }
    let retry = null
    let alive = true
    setStatus('connecting')
    const img = new Image()
    img.onload = () => alive && setStatus('live')
    img.onerror = () => {
      if (!alive) return
      setStatus('retrying')
      retry = setTimeout(() => setAttempt((n) => n + 1), 2500)
    }
    img.src = `${url}${url.includes('?') ? '&' : '?'}t=${Date.now()}`
    return () => {
      alive = false
      clearTimeout(retry)
      img.onload = null
      img.onerror = null
      img.src = ''
    }
  }, [url, active, attempt])

  return status
}
