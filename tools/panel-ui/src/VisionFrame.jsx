import { useLayoutEffect, useRef, useState } from 'react'

// Image and marks share one SVG coordinate system, so contain/letterboxing
// applies to the pair, not to two independently sized layers.
export default function VisionFrame({ frame }) {
  const { width, height, src, objects } = frame
  const element = useRef(null)
  const [scale, setScale] = useState(1)
  useLayoutEffect(() => {
    const measure = () => {
      const bounds = element.current.getBoundingClientRect()
      const next = Math.min(bounds.width / width, bounds.height / height)
      if (Number.isFinite(next) && next > 0) setScale(next)
    }
    measure()
    const observer = new ResizeObserver(measure)
    observer.observe(element.current)
    return () => observer.disconnect()
  }, [width, height])
  const fontSize = 14 / scale
  return (
    <svg ref={element} className="vision-frame" viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="xMidYMid meet"
      role="img" aria-label="Камера робота · YOLO">
      <image href={src} width={width} height={height} />
      {objects.map(({ label, confidence, box }, index) => {
        const [x1, y1, x2, y2] = box
        return <g key={index}>
          <rect x={x1 * width} y={y1 * height} width={(x2 - x1) * width} height={(y2 - y1) * height} />
          <text x={x1 * width + 3 / scale} y={Math.max(fontSize, y1 * height - 4 / scale)} fontSize={fontSize}
            style={{ strokeWidth: 2 / scale }}>
            {`${label} ${Math.round(confidence * 100)}%`}
          </text>
        </g>
      })}
    </svg>
  )
}
