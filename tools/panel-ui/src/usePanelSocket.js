import { useEffect, useRef, useState } from 'react'

// One push channel for everything the server streams. Reconnects with a
// capped backoff; every timer and listener is released on unmount or on
// server change, so no leaked socket survives a hot reload.
export default function usePanelSocket(onMessage) {
  const [connected, setConnected] = useState(false)
  const handler = useRef(onMessage)
  handler.current = onMessage
  const socketRef = useRef(null)

  useEffect(() => {
    let closed = false
    let retry = null
    let attempts = 0

    const connect = () => {
      if (closed) return
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      const ws = new WebSocket(`${proto}://${location.host}/ws`)
      socketRef.current = ws
      ws.onopen = () => {
        attempts = 0
        setConnected(true)
      }
      ws.onmessage = (event) => {
        try {
          handler.current(JSON.parse(event.data))
        } catch {
          // malformed frame: ignore, the connection stays usable
        }
      }
      ws.onclose = () => {
        setConnected(false)
        socketRef.current = null
        if (!closed) {
          attempts += 1
          const delay = Math.min(500 * 2 ** attempts, 5000)
          retry = setTimeout(connect, delay)
        }
      }
    }
    connect()
    return () => {
      closed = true
      clearTimeout(retry)
      const ws = socketRef.current
      if (ws) {
        ws.onclose = null
        ws.close()
      }
    }
  }, [])

  const send = (msg) => {
    const ws = socketRef.current
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg))
  }
  return { connected, send }
}
