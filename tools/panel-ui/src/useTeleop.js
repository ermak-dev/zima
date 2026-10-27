import { useEffect, useRef, useState } from 'react'

// Teleop: a held key publishes at 150 ms; blur, hidden tab, Space or keyup
// stop immediately. The interval exists only while keys are held, so an
// idle panel has zero running timers. The linear limit is a UI-side scale
// factor only; the server keeps clamping to its own hard limits.
const KEYS = {
  ArrowUp: ['v', 1], KeyW: ['v', 1], ArrowDown: ['v', -1], KeyS: ['v', -1],
  ArrowLeft: ['w', 1], KeyA: ['w', 1], ArrowRight: ['w', -1], KeyD: ['w', -1],
}
const LIMITS = { v: 0.1, w: 0.4 }

export default function useTeleop(send, linearLimit = LIMITS.v) {
  const held = useRef(new Set())
  const [active, setActive] = useState(false)
  const sendRef = useRef(send)
  sendRef.current = send
  const limitsRef = useRef({ v: linearLimit, w: linearLimit / LIMITS.v * LIMITS.w })
  limitsRef.current = { v: linearLimit, w: linearLimit / LIMITS.v * LIMITS.w }

  const publish = () => {
    const cmd = { v: 0, w: 0 }
    for (const code of held.current) {
      const [axis, dir] = KEYS[code]
      cmd[axis] = dir * limitsRef.current[axis]
    }
    sendRef.current(cmd)
  }

  useEffect(() => {
    const isTyping = (target) =>
      ['SELECT', 'INPUT', 'BUTTON', 'TEXTAREA'].includes(target?.tagName)

    const onKey = (event, add) => {
      if (event.code === 'Space') {
        event.preventDefault()
        held.current.clear()
        setActive(false)
        publish()
        return
      }
      if (!KEYS[event.code] || (add && isTyping(event.target))) return
      if (!add && !held.current.has(event.code)) return
      event.preventDefault()
      if (add) held.current.add(event.code)
      else held.current.delete(event.code)
      setActive(held.current.size > 0)
      publish()
    }
    const keydown = (e) => onKey(e, true)
    const keyup = (e) => onKey(e, false)
    const stop = () => {
      if (held.current.size) {
        held.current.clear()
        setActive(false)
        publish()
      }
    }
    const onBlur = () => stop()
    const onVisibility = () => { if (document.hidden) stop() }
    window.addEventListener('keydown', keydown)
    window.addEventListener('keyup', keyup)
    window.addEventListener('blur', onBlur)
    document.addEventListener('visibilitychange', onVisibility)
    return () => {
      window.removeEventListener('keydown', keydown)
      window.removeEventListener('keyup', keyup)
      window.removeEventListener('blur', onBlur)
      document.removeEventListener('visibilitychange', onVisibility)
    }
  }, [])

  // Refresh loop only while a key is held; cleanup clears it.
  useEffect(() => {
    if (!active) return
    const id = setInterval(publish, 150)
    return () => clearInterval(id)
  }, [active])

  const stop = () => {
    held.current.clear()
    setActive(false)
    publish()
  }
  return { stop, active }
}
