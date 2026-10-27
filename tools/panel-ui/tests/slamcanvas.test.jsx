import { describe, it, expect } from 'vitest'
import { parseGrid, cellRGBA, gridTransform } from '../src/LidarCanvas.jsx'

describe('parseGrid', () => {
  it('decodes probabilities and unknowns', () => {
    const g = parseGrid('50,-1,0,100', 2, 2)
    expect([...g]).toEqual([50, 255, 0, 100])
  })
  it('clamps out-of-range values and tolerates short input', () => {
    const g = parseGrid('300,-5,abc', 3, 1)
    expect([...g]).toEqual([100, 255, 255])
    expect([...parseGrid('', 1, 1)]).toEqual([0])
  })
})

// Cells carry occupancy percent: walls dark, free space light, unknown and
// undecided cells transparent so the canvas background shows through.
describe('cellRGBA', () => {
  it('draws walls dark and opaque', () => {
    const [r, g, b, a] = cellRGBA(100)
    expect(a).toBeGreaterThan(200)
    expect(r + g + b).toBeLessThan(250)
  })
  it('draws free space light and translucent', () => {
    const [, , , a] = cellRGBA(0)
    expect(a).toBeGreaterThan(0)
    expect(a).toBeLessThan(cellRGBA(100)[3])
  })
  it('leaves unknown and undecided cells transparent', () => {
    expect(cellRGBA(255)).toBeNull()
    expect(cellRGBA(50)).toBeNull()
  })
})

// Image pixel (u right, v down; row 0 = highest y) -> map -> base_link
// (inverse SLAM pose) -> screen, as one canvas affine transform.
describe('gridTransform', () => {
  const view = { W: 200, H: 100, k: 10, ox: 0, oy: 0 }
  const grid = { x_min: 0, y_min: 0, res: 1, w: 1, h: 1 }
  const apply = ([a, b, c, d, e, f], u, v) => [a * u + c * v + e, b * u + d * v + f]
  const close = (p, q) => {
    expect(p[0]).toBeCloseTo(q[0], 6)
    expect(p[1]).toBeCloseTo(q[1], 6)
  }

  it('places the map origin at the robot when the robot sits on it', () => {
    const m = gridTransform(grid, { x: 0, y: 0, yaw: 0 }, view)
    close(apply(m, 0, 1), [100, 50])
    close(apply(m, 1, 0), [110, 40])
  })
  it('shifts the map opposite to the robot translation', () => {
    const m = gridTransform(grid, { x: 1, y: 0, yaw: 0 }, view)
    close(apply(m, 0, 1), [90, 50])
  })
  it('rotates map points into the robot frame', () => {
    // Robot faces map +y: the map point (0,1) is 1 m straight ahead (+x).
    const m = gridTransform(grid, { x: 0, y: 0, yaw: Math.PI / 2 }, view)
    close(apply(m, 0, 0), [110, 50])
  })
  it('honors a panned view', () => {
    const m = gridTransform(grid, { x: 0, y: 0, yaw: 0 }, { ...view, ox: 1, oy: -1 })
    close(apply(m, 0, 1), [90, 40])
  })
})
