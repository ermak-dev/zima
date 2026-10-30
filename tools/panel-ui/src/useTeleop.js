import { useEffect, useRef, useState } from 'react'

// Teleop: a held key publishes at 150 ms; blur, hidden tab, Space or keyup
// stop immediately. The interval exists only while keys are held, so an
// idle panel has zero running timers. Speed limits are independent UI-side
// settings; the server keeps clamping to its own hard limits.
const KEYS = {
  ArrowUp: ['v', 1], KeyW: ['v', 1], ArrowDown: ['v', -1], KeyS: ['v', -1],
  ArrowLeft: ['w', 1], KeyA: ['w', 1], ArrowRight: ['w', -1], KeyD: ['w', -1],
}
export const LIMITS = { v: 0.5, w: 3.14 }
export const DEFAULT_LINEAR_SPEED = 0.25
export const DEFAULT_ANGULAR_SPEED = 1.57

export default function useTeleop(send, linearLimit = DEFAULT_LINEAR_SPEED, angularLimit = DEFAULT_ANGULAR_SPEED, blocked = false) {
  const held = useRef(new Set())
  const [active, setActive] = useState(false)
  const sendRef = useRef(send)
  sendRef.current = send
  const blockedRef = useRef(blocked)
  blockedRef.current = blocked
  const speed = Number.isFinite(linearLimit) ? Math.max(0, Math.min(linearLimit, LIMITS.v)) : 0
  const limitsRef = useRef({ v: 0, w: 0 })
  limitsRef.current = {
    v: speed,
    w: Number.isFinite(angularLimit) ? Math.max(0, Math.min(angularLimit, LIMITS.w)) : 0,
  }

  const publish = () => {
    if (blockedRef.current) return
    const cmd = { v: 0, w: 0 }
    for (const code of held.current) {
      const [axis, dir] = KEYS[code]
      cmd[axis] = dir * limitsRef.current[axis]
    }
    sendRef.current(cmd)
  }

  useEffect(() => {
    const isTyping = (target, code) => {
      // Allow WASD on range sliders (for robot control), but not arrows (they adjust the slider)
      if (target?.tagName === 'INPUT' && target.type === 'range') {
        return !['KeyW', 'KeyA', 'KeyS', 'KeyD'].includes(code)
      }
      return ['SELECT', 'INPUT', 'BUTTON', 'TEXTAREA'].includes(target?.tagName)
    }

    const onKey = (event, add) => {
      if (blockedRef.current) return
      if (event.code === 'Space') {
        if (isTyping(event.target, event.code) || event.target?.closest?.('[contenteditable]:not([contenteditable="false"])')) return
        event.preventDefault()
        held.current.clear()
        setActive(false)
        publish()
        return
      }
      if (!KEYS[event.code] || (add && isTyping(event.target, event.code))) return
      if (!add && !held.current.has(event.code)) return
      if (add && event.repeat && !held.current.has(event.code)) return
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

  useEffect(() => {
    if (!blocked) return
    held.current.clear()
    setActive(false)
  }, [blocked])

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
