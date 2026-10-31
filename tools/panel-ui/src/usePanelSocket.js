import { useEffect, useRef, useState } from 'react'
import wsChannel from './wsChannel.js'

// One push channel for everything the server streams. Reconnects with the
// panel's own capped backoff; every timer and listener is released on unmount
// or on server change, so no leaked socket survives a hot reload.
export default function usePanelSocket(onMessage) {
  const [connected, setConnected] = useState(false)
  const handler = useRef(onMessage)
  handler.current = onMessage
  const channelRef = useRef(null)

  useEffect(() => {
    let attempts = 0
    const channel = wsChannel('/ws', {
      onOpen: () => {
        attempts = 0
        setConnected(true)
      },
      onMessage: (ws, event) => {
        try {
          handler.current(JSON.parse(event.data))
        } catch {
          // malformed frame: ignore, the connection stays usable
        }
      },
      onClosed: () => {
        setConnected(false)
        attempts += 1
        return Math.min(500 * 2 ** attempts, 5000)
      },
    })
    channelRef.current = channel
    return () => {
      channelRef.current = null
      channel.stop()
    }
  }, [])

  const send = (msg) => channelRef.current?.send(msg)
  return { connected, send }
}
