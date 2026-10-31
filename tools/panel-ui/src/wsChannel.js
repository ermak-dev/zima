// Shared WebSocket lifecycle mechanics only: URL building, connecting,
// detaching handlers on close, consumer-driven backoff and cleanup. Message
// semantics, decode logic and backoff policy stay with the consumers, so the
// control and video channels keep independent connections.
export default function wsChannel(path, { onOpen, onMessage, onClosed, onError, onAttempt }) {
  let stopped = false
  let retry = null
  let socket = null

  const connect = () => {
    if (stopped) return
    onAttempt?.()
    const proto = location.protocol === 'https:' ? 'wss' : 'ws'
    const ws = new WebSocket(`${proto}://${location.host}${path}`)
    socket = ws
    ws.onopen = onOpen ? () => onOpen(ws) : null
    ws.onmessage = onMessage ? (event) => onMessage(ws, event) : null
    ws.onerror = onError ? () => onError(ws) : null
    ws.onclose = () => {
      if (socket !== ws) return
      detach(ws)
      socket = null
      const delay = stopped ? undefined : onClosed?.(ws)
      if (!stopped && Number.isFinite(delay) && delay >= 0) retry = setTimeout(connect, delay)
    }
  }
  const detach = (ws) => {
    ws.onopen = null
    ws.onmessage = null
    ws.onclose = null
    ws.onerror = null
  }
  const stop = () => {
    stopped = true
    clearTimeout(retry)
    retry = null
    if (socket) {
      detach(socket)
      socket.close()
      socket = null
    }
  }
  connect()
  return {
    get socket() { return socket },
    send: (msg) => {
      if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(msg))
    },
    stop,
  }
}
