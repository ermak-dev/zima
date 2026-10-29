// FEATURE: layout focused on the map and lidar.
// SPEC: no SLAM section, no service channels; lidar visibility checkboxes;
// motor controls live in the bottom footer next to speed sliders.
import { describe, it, expect, vi, afterEach } from 'vitest'
import { render, screen, fireEvent, act, cleanup } from '@testing-library/react'
import App from '../src/App.jsx'

let lastHandler
vi.mock('../src/usePanelSocket.js', () => ({ default: (handler) => {
  lastHandler = handler
  return { connected: true, send: vi.fn() }
} }))
afterEach(() => cleanup())

describe('Panel layout', () => {
  it('shows lidar and SLAM layer checkboxes instead of a SLAM section', () => {
    const { container } = render(<App />)
    expect(screen.getByRole('checkbox', { name: /скан лидара/i })).toBeTruthy()
    expect(screen.getByRole('checkbox', { name: /карта sl/i })).toBeTruthy()
    expect(screen.queryByLabelText('SLAM')).toBeNull()
    expect(container.textContent).not.toContain('сопровождение')
    expect(container.textContent).not.toContain('построение карты')
  })

  it('checkboxes toggle scan points and map layers off by default-off state', () => {
    const { container } = render(<App />)
    const lidarBox = screen.getByRole('checkbox', { name: /скан лидара/i })
    const mapBox = screen.getByRole('checkbox', { name: /карта sl/i })
    expect(lidarBox.checked).toBe(true)
    expect(mapBox.checked).toBe(true)
    fireEvent.click(lidarBox)
    expect(lidarBox.checked).toBe(false)
    fireEvent.click(mapBox)
    expect(mapBox.checked).toBe(false)
  })

  it('moves motor controls into the footer', () => {
    render(<App />)
    const footer = document.querySelector('footer')
    const motors = footer?.querySelector('#motors')
    expect(motors).toBeTruthy()
    expect(footer.querySelectorAll('input[type="range"]').length).toBeGreaterThanOrEqual(5)
  })

  it('removes service channels entirely', () => {
    const { container } = render(<App />)
    expect(screen.queryByRole('group', { name: 'Служебные каналы' })).toBeNull()
    expect(container.textContent).not.toContain('ir:6')
    expect(container.textContent).not.toContain('motor:0')
    expect(container.textContent).not.toContain('подписка отключена')
  })

  it('keeps chassis sensor widgets in the sensors panel', () => {
    const { container } = render(<App />)
    expect(screen.getByRole('img', { name: 'Компас' })).toBeTruthy()
    expect(screen.getByRole('img', { name: 'Гироскоп' })).toBeTruthy()
    expect(screen.getByRole('img', { name: 'Положение одометрии' })).toBeTruthy()
    expect(container.querySelector('.sensor-readings')).toBeTruthy()
  })
})
