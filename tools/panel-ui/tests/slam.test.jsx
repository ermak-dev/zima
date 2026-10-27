import { describe, it, expect } from 'vitest'
import { render, screen } from '@testing-library/react'
import SlamStatus from '../src/SlamStatus.jsx'

// The slam block appears only with slam state; mode text and quality are
// driven verbatim from the server payload, unknown stays "—".
describe('SlamStatus', () => {
  it('renders nothing without slam state', () => {
    const { container } = render(<SlamStatus slam={undefined} />)
    expect(container.textContent).toBe('')
  })

  it('shows mapping mode with pose and quality', () => {
    render(<SlamStatus slam={{ mode: 'mapping', pose: { x: 1.2, y: -0.4, yaw: 0.5 }, quality: 1 }} />)
    expect(screen.getByText(/SLAM/)).toBeTruthy()
    expect(screen.getByText('построение карты')).toBeTruthy()
    expect(screen.getByText(/1\.2/)).toBeTruthy()
    expect(screen.getByText(/-0\.4/)).toBeTruthy()
  })

  it('marks lost mode as blocked', () => {
    render(<SlamStatus slam={{ mode: 'lost', pose: null, quality: 0 }} />)
    expect(screen.getByText(/потеря локализации/)).toBeTruthy()
  })

  it('shows dashes for unknown pose fields', () => {
    render(<SlamStatus slam={{ mode: 'localize', pose: null, quality: null }} />)
    expect(screen.getAllByText(/—/).length).toBeGreaterThanOrEqual(4)
  })
})
