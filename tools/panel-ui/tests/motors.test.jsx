// FEATURE: user-facing cleaning motor controls replacing SLAM debug output.
// SPEC: fan/main/side duty sliders 0..100 send {"type":"motors",...} through
// the panel socket; missing state shows dashes; no motor telemetry invented.
import { describe, it, expect, vi, afterEach } from 'vitest'
import { render, screen, fireEvent, cleanup } from '@testing-library/react'
import MotorsPanel from '../src/MotorsPanel.jsx'

afterEach(() => cleanup())
const send = vi.fn()

describe('MotorsPanel', () => {
  it('renders three duty controls with server state values', () => {
    render(<MotorsPanel motors={{ fan: 40, main: 60, side: 0 }} send={send} />)
    expect(screen.getByLabelText(/вентилятор/i).value).toBe('40')
    expect(screen.getByLabelText(/основная щётка/i).value).toBe('60')
    expect(screen.getByLabelText(/боковая щётка/i).value).toBe('0')
  })

  it('shows dashes when motor state is unknown', () => {
    const { container } = render(<MotorsPanel motors={undefined} send={send} />)
    expect(container.textContent).toContain('—')
  })

  it('sends a motors command on change', () => {
    render(<MotorsPanel motors={{ fan: 0, main: 0, side: 0 }} send={send} />)
    fireEvent.change(screen.getByLabelText(/вентилятор/i), { target: { value: '55' } })
    expect(send).toHaveBeenCalledWith({ type: 'motors', fan: 55, main: 0, side: 0 })
  })

  it('clamps input outside 0..100 to the bounds', () => {
    render(<MotorsPanel motors={{ fan: 0, main: 0, side: 0 }} send={send} />)
    const fan = screen.getByLabelText(/вентилятор/i)
    fireEvent.change(fan, { target: { value: '150' } })
    expect(send).toHaveBeenCalledWith(expect.objectContaining({ fan: 100 }))
  })
})
