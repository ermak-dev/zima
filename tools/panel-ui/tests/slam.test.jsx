import { describe, it, expect, vi, afterEach } from 'vitest'
import { render, screen, cleanup, act } from '@testing-library/react'
import SlamStatus from '../src/SlamStatus.jsx'

// FEATURE: Actual ICP diagnostics and result-identity freshness, not fabricated trust.
// SPEC: Unknowns remain unknown; unrelated telemetry never rejuvenates a pose;
// source stamps are not wall time; stale/error/lost and mismatched IDs are explicit.
// IMPLEMENTATION: shared monotonic identity clock and measured diagnostics, no trust score.
// RESULT: IMPLEMENTED — focused assertions failed before source edits, now pass.
// STATUS: DONE
afterEach(() => { cleanup(); vi.useRealTimers() })
const result = {
  mode: 'mapping', tracking: 'tracking', pose: { x: 1, y: 2, yaw: 0 },
  scan_id: 10, scan_stamp: 12.5, age_ms: 20, max_age_ms: 500, stale: false,
  source_time_verified: false, odom_association: 'receive_order', odom_skew_ms: null,
  dropped_scans: 3,
  diagnostics: { input_points: 320, used_points: 150, reference_points: 610,
    accepted: false, reason: 'low_goodness', goodness: 0.21, iterations: 8,
    duration_ms: 13.4, residual_rms: null, covariance: null, correction: null },
}

describe('SLAM user-facing status (debug details removed)', () => {
  it('shows mode, tracking, pose, quality and age without debug internals', () => {
    const { container } = render(<SlamStatus slam={result} scan={{ id: 10 }} />)
    for (const text of ['сопровождение', '1.00', '2.00', '0°']) {
      expect(container.textContent).toContain(text)
    }
    for (const debug of ['low_goodness', '320', 'receive_order', 'input', 'RMS', 'итерации',
      'goodness', 'MCL', 'deskew', 'одометрия', 'пропущено', 'время не подтверждено',
      'соответствия', 'ICP мс', 'опора']) {
      expect(container.textContent).not.toContain(debug)
    }
  })

  it('keeps user-relevant warnings: paused mapping, stale, mismatch, error', () => {
    const { container } = render(<SlamStatus slam={{ ...result,
      diagnostics: { ...result.diagnostics, mapping_paused: true }, stale: true, error: 'adapter timeout' }}
      scan={{ id: 11 }} />)
    expect(container.textContent).toContain('пополнение карты приостановлено')
    expect(container.textContent).toContain('устарело')
    expect(container.textContent).toContain('скан и поза не совпадают')
    expect(container.textContent).toContain('adapter timeout')
  })

  it('ages a result despite repeated unrelated state and rejuvenates only on a new identity', () => {
    vi.useFakeTimers()
    const { container, rerender } = render(<SlamStatus slam={result} scan={{ id: 10 }} />)
    act(() => vi.advanceTimersByTime(300))
    rerender(<SlamStatus slam={{ ...result, dropped_scans: 4 }} scan={{ id: 10 }} />)
    act(() => vi.advanceTimersByTime(300))
    expect(container.textContent).toContain('устарело')
    rerender(<SlamStatus slam={{ ...result, scan_id: 11 }} scan={{ id: 11 }} />)
    expect(container.textContent).not.toContain('устарело')
    act(() => vi.advanceTimersByTime(600))
    rerender(<SlamStatus slam={{ ...result, scan_id: 11, scan_stamp: 13 }} scan={{ id: 11 }} />)
    expect(container.textContent).not.toContain('устарело')
    cleanup()
    expect(vi.getTimerCount()).toBe(0)
  })
})

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
