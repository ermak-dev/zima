import { useEffect, useState } from 'react'

export function compassYaw(compass) {
  if (compass?.source !== 'st480-evdev' || compass.status !== 'uncalibrated' ||
    compass.calibrated !== false || !Number.isFinite(compass.stamp) || compass.stamp <= 0 ||
    !Number.isFinite(compass.x) || !Number.isFinite(compass.y) ||
    (compass.x === 0 && compass.y === 0)) return null
  return Number.isFinite(compass.yaw) ? compass.yaw : null
}

export default function useCompass(compass, connected) {
  const [expiredStamp, setExpiredStamp] = useState(null)
  const stamp = compass?.stamp
  const source = compass?.source
  const status = compass?.status
  useEffect(() => {
    if (source !== 'st480-evdev' || status !== 'uncalibrated' || !Number.isFinite(stamp)) return
    // Other telemetry and reopening the instrument must not renew a magnetic sample.
    const timer = setTimeout(() => setExpiredStamp(stamp), 1500)
    return () => clearTimeout(timer)
  }, [stamp, source, status])
  if (!connected) return { ...compass, yaw: null, status: 'disconnected' }
  if (stamp != null && stamp === expiredStamp) return { ...compass, yaw: null, status: 'stale' }
  return compass
}
