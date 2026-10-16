/**
 * Thin seam over wall-clock time, injected wherever code needs to reason
 * about elapsed time (currently just TeleopService's per-command deadline)
 * so unit tests can advance time deterministically instead of sleeping.
 */
export interface Clock {
  now(): number;
}

export class SystemClock implements Clock {
  now(): number {
    return Date.now();
  }
}
