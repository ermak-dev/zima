import { useEffect, useRef, useState } from 'react'

// Decoded occupancy images live in a ref, never in state: state only holds
// the small version stamp that triggers a redraw. Bitmaps are explicitly
// closed after each swap, and the observer disconnects on unmount.
export default function useOccupancyMap(onReplace) {
  const bitmapRef = useRef(null)
  const metaRef = useRef(null)
  const [, bump] = useStateTick()
  const onReplaceRef = useRef(onReplace)
  onReplaceRef.current = onReplace

  useEffect(() => () => {
    bitmapRef.current?.close?.()
    bitmapRef.current = null
  }, [])

  const applyMap = (grid) => {
    if (grid.version === undefined) {
      bitmapRef.current?.close?.()
      bitmapRef.current = null
      metaRef.current = null
      bump()
      return
    }
    if (!grid.image || grid.version === metaRef.current?.version) return
    // Metadata applies immediately: frame/pose logic must not wait for decode.
    metaRef.current = { ...grid, bitmapReady: false, error: null }
    const img = new Image()
    img.onload = () => {
      const old = bitmapRef.current
      bitmapRef.current = img
      metaRef.current = { ...metaRef.current, bitmapReady: true }
      old?.close?.()
      onReplaceRef.current?.(grid)
      bump()
    }
    img.onerror = () => {
      metaRef.current = { ...metaRef.current, error: 'Не удалось загрузить карту' }
      bump()
    }
    img.src = grid.image
  }

  return { applyMap, get bitmap() { return bitmapRef.current }, get meta() { return metaRef.current } }
}

function useStateTick() {
  const ref = useRef(0)
  const set = useRef(null)
  const [tick, setTickState] = useState(0)
  set.current = setTickState
  return [tick, () => set.current((n) => n + 1)]
}
