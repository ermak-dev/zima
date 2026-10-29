import { useEffect, useRef, useState } from 'react'

// Source stamps identify results, never wall-clock time. Repeated state messages
// may raise a known age, but cannot reset the client-side monotonic clock.
export default function useSlamState(slam, scan) {
  const [, tick] = useState(0)
  const clock = useRef(null)
  const id = Number.isFinite(slam?.scan_id) ? slam.scan_id : null
  const stamp = Number.isFinite(slam?.scan_stamp) ? slam.scan_stamp : null
  const sequence = Number.isFinite(slam?.scan_sequence) ? slam.scan_sequence : null
  const reportedAge = Number.isFinite(slam?.age_ms) && slam.age_ms >= 0 ? slam.age_ms : null
  const timed = id !== null && reportedAge !== null
  const now = performance.now()
  if (!timed) {
    clock.current = null
  } else if (!clock.current || clock.current.id !== id || clock.current.stamp !== stamp || clock.current.sequence !== sequence) {
    clock.current = { id, stamp, sequence, age: reportedAge, received: now }
  } else if (reportedAge > clock.current.age + now - clock.current.received) {
    clock.current = { id, stamp, sequence, age: reportedAge, received: now }
  }
  const ageMs = clock.current ? clock.current.age + Math.max(0, now - clock.current.received) : null
  const maxAgeMs = Number.isFinite(slam?.max_age_ms) && slam.max_age_ms >= 0 ? slam.max_age_ms : 500
  const stale = slam?.stale === true || (ageMs !== null && ageMs > maxAgeMs)
  const mismatch = (Number.isFinite(scan?.sequence) && sequence !== null && scan.sequence !== sequence) ||
    (Number.isFinite(scan?.id) && id !== null && scan.id !== id) ||
    (Number.isFinite(scan?.stamp) && stamp !== null && scan.stamp !== stamp)
  const pose = slam?.pose
  const validPose = pose && [pose.x, pose.y, pose.yaw].every(Number.isFinite)
  const failed = !!slam?.error || ['error', 'lost'].includes(slam?.tracking) || slam?.mode === 'lost'
  const aligned = !!validPose && !stale && !failed && !mismatch
  const matched = aligned && Number.isFinite(scan?.id) && id !== null && scan.id === id

  useEffect(() => {
    if (!timed) return
    const timer = setInterval(() => tick((n) => n + 1), 100)
    return () => clearInterval(timer)
  }, [timed])

  return { ageMs, maxAgeMs, stale, mismatch, failed, aligned, matched }
}
