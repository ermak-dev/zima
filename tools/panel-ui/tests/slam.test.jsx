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

describe('SLAM diagnostic contract', () => {
  it('shows real counts, rejected match and reason without calling goodness trust', () => {
    const { container } = render(<SlamStatus slam={result} scan={{ id: 10 }} />)
    for (const text of ['320', '150', '610', 'low_goodness', '0.21', '13.4', 'receive_order']) {
      expect(container.textContent).toContain(text)
    }
    expect(container.textContent).toContain('отклонено')
    expect(container.textContent).toContain('время не подтверждено')
    expect(container.textContent).toContain('пропущено')
    expect(container.textContent).not.toContain('доверие')
  })

  it('shows measured correspondence counts and unavailable deskew explicitly', () => {
    const { container } = render(<SlamStatus slam={{ ...result,
      deskew: { requested: true, applied: false, status: 'unavailable' },
      diagnostics: { ...result.diagnostics, correspondences: 147, correspondence_threshold_m: 0.1, mapping_paused: true } }} />)
    expect(container.textContent).toContain('пополнение карты приостановлено')
    expect(container.textContent).toContain('147')
    expect(container.textContent).toContain('deskew недоступен')
  })

  it('keeps the MCL spread proxy separate from ICP goodness', () => {
    const { container } = render(<SlamStatus slam={{ ...result, mode: 'localize',
      localization_quality: 0.95, diagnostics: { goodness: null, reason: 'mcl_update', accepted: false } }} />)
    expect(container.textContent).toContain('MCL 0.95')
    expect(container.textContent).toContain('goodness —')
  })

  it('does not infer accepted, goodness or timing from missing metadata or old quality', () => {
    const { container } = render(<SlamStatus slam={{ mode: 'mapping', quality: 1, diagnostics: { goodness: null, accepted: null } }} />)
    expect(container.textContent).toContain('ICP —')
    expect(container.textContent).toContain('возраст неизвестен')
    expect(container.textContent).toContain('время не подтверждено')
    expect(container.textContent).not.toContain('1.00')
    expect(container.textContent).not.toContain('свежее')
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

  it('labels mismatched scans, server stale and errors immediately', () => {
    const { container, rerender } = render(<SlamStatus slam={result} scan={{ id: 11 }} />)
    expect(container.textContent).toContain('скан и поза не совпадают')
    rerender(<SlamStatus slam={{ ...result, stale: true }} scan={{ id: 10 }} />)
    expect(container.textContent).toContain('устарело')
    rerender(<SlamStatus slam={{ ...result, tracking: 'error', error: 'adapter timeout' }} />)
    expect(container.textContent).toContain('adapter timeout')
  })

  it('shows accepted ICP and verified timing only when explicitly reported', () => {
    const { container } = render(<SlamStatus slam={{ ...result, source_time_verified: true,
      odom_association: 'interpolated', odom_skew_ms: 4.2,
      diagnostics: { ...result.diagnostics, accepted: true, goodness: 0.82 } }} />)
    expect(container.textContent).toContain('принято')
    expect(container.textContent).toContain('0.82')
    expect(container.textContent).toContain('время подтверждено')
    expect(container.textContent).toContain('4.2')
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
